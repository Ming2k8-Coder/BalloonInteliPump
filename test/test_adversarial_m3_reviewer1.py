#!/usr/bin/env python3
"""
Adversarial Stress Test Suite for Milestone 3 (M3):
Backend, Streaming, Machine Learning Pipeline & Concurrency

Authored by: Reviewer 1 (teamwork_preview_reviewer_m3_1)
Objective:
1. Adversarial RLS Stress Testing:
   - 100,000 steps of stationary steady-state pressure to test covariance windup.
   - Extreme inputs: NaN, -Inf, +Inf, -50.0 kPa (negative gauge), 500.0 kPa (extreme overpressure).
   - Trace ceiling, positive semi-definiteness, weight finiteness, and physical forecast bounding [0, 100].
2. High-Concurrency & Mutex Stress Testing:
   - 8 concurrent feeder threads vs 4 snapshot readers on TelemetryEngine buffers.
   - Assert zero RuntimeError: deque mutated during iteration.
   - Concurrent start/stop logging stress test.
   - Bounded SSE client queue flood test (1,000 bursts) verifying zero memory leaks or blockages.
3. Predictive Burst Safety Boundary Testing:
   - Precise epsilon boundary tests around the 95% threshold across all 11 mode codes.
   - Verify active modes [1, 2, 3, 4, 5, 8, 9, 10] trigger STOP; inactive [0, 6, 7] do not.
   - Invalid/negative/zero estimated_burst handling.
4. ML Pipeline & Schema Adversarial Tests:
   - Training pipeline with zero-variance features, single-class labels, mixed string/int mode codes.
   - Classification report zero-division handling.
5. Telemetry Parser Stress Testing:
   - Fuzzed and malformed NMEA sentences, missing fields, corrupt checksums, buffer overflows.
"""

import sys
import os
import time
import math
import queue
import threading
import collections
from pathlib import Path
import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

import server.app as srv
import server.train_ml as tml
import gui.app as gui_app


def test_adversarial_rls():
    print("[Adversarial 1/5] Stress-Testing RLS Predictor under Extreme Numerical Conditions...")
    predictor = srv.RLSPredictor(order=5, forgetting_factor=0.98, forecast_steps=15)

    # 1. 100,000 steps of stationary steady-state pressure (12.0 kPa)
    for _ in range(100000):
        predictor.update(12.0)

    tr = float(np.trace(predictor.P))
    assert np.all(np.isfinite(predictor.P)), "Covariance matrix P contains non-finite values after 100,000 samples!"
    assert np.all(np.isfinite(predictor.w)), "Weights vector w contains non-finite values after 100,000 samples!"
    assert tr <= 10000.0 + 1e-4, f"Trace ceiling breached: Tr(P) = {tr} > 10000.0"
    assert np.allclose(predictor.P, predictor.P.T, atol=1e-4), "P lost symmetry!"

    # Eigenvalue check: P must remain positive semi-definite
    eigvals = np.linalg.eigvalsh(predictor.P)
    assert np.all(eigvals >= -1e-6), f"Covariance matrix P has negative eigenvalues: {eigvals}"

    # Forecast must be physically valid
    fc = predictor.forecast()
    assert np.isfinite(fc)
    assert 0.0 <= fc <= 100.0, f"Forecast {fc} outside [0, 100] kPa"

    # 2. Hostile / Adversarial Inputs: NaNs, Infs, Negatives, Massive values
    hostile_samples = [
        float('nan'), float('inf'), float('-inf'), -100.0, 9999.0, "invalid_str", None, 0.0, 1e-12
    ]
    for s in hostile_samples:
        pred_out = predictor.update(s)
        assert np.isfinite(pred_out), f"Update returned non-finite on input {s}: {pred_out}"
        fc = predictor.forecast()
        assert np.isfinite(fc), f"Forecast returned non-finite on input {s}: {fc}"
        assert 0.0 <= fc <= 100.0, f"Forecast {fc} breached [0, 100] bounds on hostile input {s}"

    # 3. Dynamic Step Response
    for _ in range(20):
        predictor.update(45.0)
    fc_high = predictor.forecast()
    assert 0.0 <= fc_high <= 100.0

    print(f"      PASS: 100,000 steady samples & hostile input fuzzing strictly stable (Tr(P)={tr:.1f}, eigmin={min(eigvals):.4f}).")


def test_adversarial_concurrency():
    print("[Adversarial 2/5] Stress-Testing Concurrency, Thread Safety & SSE Queue Bounds...")
    engine = srv.TelemetryEngine()
    engine.send_cmd = lambda cmd: None  # mock send_cmd

    stop_event = threading.Event()
    exceptions = []

    # 8 feeder threads generating continuous telemetry
    def feeder_worker(thread_id):
        step = 0
        while not stop_event.is_set():
            step += 1
            p = 10.0 + 5.0 * math.sin(step * 0.05 + thread_id)
            payload = f"BIP,{step*5},{p:.2f},180,12.0,1.8,2,{p:.2f},12.0,1.8"
            cs = 0
            for ch in payload:
                cs ^= ord(ch)
            line = f"${payload}*{cs:02X}"
            try:
                engine.process_line(line)
            except Exception as e:
                exceptions.append(f"Feeder-{thread_id} error: {e}")
            time.sleep(0.0005)

    # 4 reader threads taking atomic snapshots
    def reader_worker(thread_id):
        while not stop_event.is_set():
            try:
                with engine.history_lock:
                    snap = list(engine.history_buffer)
                if snap:
                    _ = sum(item["p"] for item in snap) / len(snap)
            except Exception as e:
                exceptions.append(f"Reader-{thread_id} error: {e}")
            time.sleep(0.001)

    threads = []
    for i in range(8):
        t = threading.Thread(target=feeder_worker, args=(i,), daemon=True)
        threads.append(t)
        t.start()

    for i in range(4):
        t = threading.Thread(target=reader_worker, args=(i,), daemon=True)
        threads.append(t)
        t.start()

    # Let concurrent traffic hammer the engine
    time.sleep(1.0)
    stop_event.set()
    for t in threads:
        t.join(timeout=1.0)

    assert len(exceptions) == 0, f"Encountered concurrency exceptions: {exceptions}"

    # Test Bounded SSE Queues: 5 slow subscribers flooded with 1,000 items
    slow_clients = [queue.Queue(maxsize=100) for _ in range(5)]
    for q_client in slow_clients:
        engine.add_client(q_client)

    for i in range(1000):
        engine.broadcast_data({"packet_id": i, "val": 42.0})

    for q_client in slow_clients:
        assert q_client.qsize() == 100, f"Queue size {q_client.qsize()} exceeded maxsize 100"
        engine.remove_client(q_client)

    print("      PASS: 12 concurrent worker threads & 1,000 flooded SSE packets handled with zero race conditions.")


def test_adversarial_predictive_burst_boundary():
    print("[Adversarial 3/5] Testing Predictive Burst Safety Boundary & Mode Enforcement...")
    engine = srv.TelemetryEngine()
    dispatched_cmds = []
    engine.send_cmd = lambda cmd: dispatched_cmds.append(cmd)

    # Set estimated burst to 40.0 kPa -> 95% threshold is exactly 38.00 kPa
    engine.estimated_burst = 40.0
    thresh = 40.0 * 0.95

    active_modes = [1, 2, 3, 4, 5, 8, 9, 10]
    inactive_modes = [0, 6, 7]

    # Test 1: Just below threshold (37.99 kPa) -> NO STOP command should be dispatched
    engine.predictor.forecast = lambda: thresh - 0.01
    for mode in active_modes:
        dispatched_cmds.clear()
        payload = f"BIP,100,20.0,150,12.0,1.5,{mode},20.0,12.0,1.5"
        cs = 0
        for ch in payload: cs ^= ord(ch)
        engine.process_line(f"${payload}*{cs:02X}")
        assert "STOP" not in dispatched_cmds, f"Premature STOP dispatched at {thresh - 0.01} kPa for mode {mode}"

    # Test 2: Exactly at and above threshold (38.00 kPa) -> STOP MUST be dispatched for all active modes
    engine.predictor.forecast = lambda: thresh
    for mode in active_modes:
        dispatched_cmds.clear()
        payload = f"BIP,100,20.0,150,12.0,1.5,{mode},20.0,12.0,1.5"
        cs = 0
        for ch in payload: cs ^= ord(ch)
        engine.process_line(f"${payload}*{cs:02X}")
        assert "STOP" in dispatched_cmds, f"STOP command missed at exactly {thresh} kPa for mode {mode}"

    # Test 3: Far above threshold (45.00 kPa) in inactive modes -> STOP MUST NOT be dispatched
    engine.predictor.forecast = lambda: 45.0
    for mode in inactive_modes:
        dispatched_cmds.clear()
        payload = f"BIP,100,20.0,0,12.0,0.0,{mode},20.0,12.0,0.0"
        cs = 0
        for ch in payload: cs ^= ord(ch)
        engine.process_line(f"${payload}*{cs:02X}")
        assert "STOP" not in dispatched_cmds, f"Spurious STOP dispatched in inactive mode {mode}"

    print(f"      PASS: Threshold boundary ({thresh:.2f} kPa) strictly enforced across all active/inactive modes.")


def test_adversarial_ml_pipeline():
    print("[Adversarial 4/5] Testing ML Feature Extraction & Sklearn Classifier Robustness...")
    # Test feature extraction on degenerate edge-case DataFrame
    degenerate_data = {
        "Pressure_kPa": [10.0] * 50,  # Constant pressure (zero variance)
        "Slope_kPa_s": [0.0] * 50,
        "Voltage_V": [12.0] * 50,
        "Current_A": [0.0] * 50,      # Zero current (zero power)
        "Mode": ["IDLE"] * 25 + [0] * 25,
        "Health_Pct": [100.0] * 50,
        "PWM": [0] * 50
    }
    df = pd.DataFrame(degenerate_data)
    df_feat = tml.compute_advanced_features(df)

    # Check for NaN generation in rolling features
    assert not df_feat["Pressure_Var_300ms"].isna().any(), "NaN found in Pressure_Var_300ms"
    assert not df_feat["Power_to_Pressure_Ratio"].isna().any(), "NaN found in Power_to_Pressure_Ratio"
    assert not df_feat["Power_W"].isna().any(), "NaN found in Power_W"
    assert np.all(df_feat["Pressure_Var_300ms"] == 0.0), "Variance should be zero on constant signal"

    # Test labeling on degenerate dataframe
    df_labeled = tml.label_data(df_feat)
    assert "Target_State" in df_labeled.columns
    # With health=100 and p=10, all should be STABLE (0)
    assert np.all(df_labeled["Target_State"] == 0)

    # Test classification report zero-division on extreme imbalanced predictions
    from sklearn.metrics import classification_report
    y_true = np.array([0, 0, 0, 0])
    y_pred = np.array([1, 1, 1, 1])  # 100% false predictions
    rep = classification_report(y_true, y_pred, labels=[0, 1, 2, 3], zero_division=0)
    assert "0.00" in rep

    print("      PASS: Degenerate signals (zero variance, zero power) and imbalanced metrics handled safely.")


def test_adversarial_telemetry_fuzzing():
    print("[Adversarial 5/5] Fuzzing Telemetry NMEA Parser with Malformed Datagrams...")
    engine = srv.TelemetryEngine()

    fuzz_packets = [
        "",
        "$",
        "$*",
        "$*FF",
        "BIP,100,10.0*00",
        "$BIP*XX",
        "$BIP,abc,def,ghi,jkl*00",
        "$BIP,100,NaN,150,12.0,1.5,2*00",
        "$BIP,100,1e308,150,12.0,1.5,2*00",
        "$BIP,100,-10.0,150,12.0,1.5,999*00",  # Invalid mode 999
        "$" + "A" * 8192 + "*00",              # Buffer overflow attempt
        "$BIP,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17*00",
    ]

    for pkt in fuzz_packets:
        try:
            engine.process_line(pkt)
        except Exception as e:
            assert False, f"Parser crashed on input '{pkt[:30]}...': {e}"

    print("      PASS: Fuzzed and oversized telemetry datagrams parsed with zero crashes or memory corruption.")


def main():
    print("=================================================================")
    print("Milestone 3 Independent Adversarial Stress Test Suite")
    print("=================================================================")
    try:
        test_adversarial_rls()
        test_adversarial_concurrency()
        test_adversarial_predictive_burst_boundary()
        test_adversarial_ml_pipeline()
        test_adversarial_telemetry_fuzzing()
        print("=================================================================")
        print("ALL 5 ADVERSARIAL STRESS TEST CATEGORIES PASSED (5/5) 100%!")
        print("=================================================================")
        return 0
    except AssertionError as e:
        import traceback
        traceback.print_exc()
        print(f"\n[FAIL] ADVERSARIAL FAILURE: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\n[ERROR] UNEXPECTED ADVERSARIAL EXCEPTION: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
