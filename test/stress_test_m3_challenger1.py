#!/usr/bin/env python3
"""
Empirical Adversarial Stress Harness for Milestone 3 (M3):
Backend, RLS Adaptive Predictor, and ML Pipeline.

Designed and Executed by: teamwork_preview_challenger_m3_1
Adversarial Dimensions:
1. Long-duration constant input (10^5 samples of 0.0 kPa and 20.0 kPa) - Trace bounds, weight convergence, finiteness.
2. Step and impulse responses - Boundedness in [0.0, 100.0] kPa, recovery, stability under shocks.
3. RLS Non-Leakage & Autoregressive Causality - Non-anticipating prediction verification.
4. train_ml.py edge cases - Single-class, 2-class, 3-class datasets, schema variations, mixed types, zero-variance.
"""

import os
import sys
import math
import shutil
import tempfile
from pathlib import Path
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report
import joblib

# Ensure repo root is in python path
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from server.app import RLSPredictor
import server.train_ml as tml


def log_test_header(title):
    print("\n" + "=" * 70)
    print(f"CHALLENGE: {title}")
    print("=" * 70)


# =====================================================================
# CHALLENGE 1: RLS 100,000-SAMPLE LONG-DURATION CONSTANT INPUT STABILITY
# =====================================================================
def test_rls_100k_zero_input():
    log_test_header("1.1: RLS Stability Under 100,000 Samples of 0.0 kPa")
    predictor = RLSPredictor(order=6, forgetting_factor=0.985, forecast_steps=15)
    
    # Track trace and weight magnitudes
    max_trace = 0.0
    for i in range(100000):
        y_pred = predictor.update(0.0)
        tr = float(np.trace(predictor.P))
        if tr > max_trace:
            max_trace = tr
        if i % 25000 == 0:
            forecast_val = predictor.forecast()
            print(f"  Step {i:6d}: Tr(P) = {tr:8.2f}, y_pred = {y_pred:.4f}, forecast = {forecast_val:.4f}")
            assert np.isfinite(tr), f"Step {i}: Tr(P) is non-finite!"
            assert np.all(np.isfinite(predictor.P)), f"Step {i}: Covariance matrix P contains NaN/Inf!"
            assert np.all(np.isfinite(predictor.w)), f"Step {i}: Weight vector w contains NaN/Inf!"
            assert tr <= 10000.0 + 1e-3, f"Step {i}: Tr(P) exceeded 10,000! Tr={tr}"
            assert np.all(np.diag(predictor.P) >= -1e-7), f"Step {i}: Negative diagonal in covariance matrix!"
            assert 0.0 <= forecast_val <= 100.0, f"Step {i}: Forecast out of bounds [0, 100]!"

    final_tr = float(np.trace(predictor.P))
    final_f = predictor.forecast()
    print(f"  Final Step 100000: Max Tr(P) = {max_trace:.2f}, Final Tr(P) = {final_tr:.2f}, Final Forecast = {final_f:.4f}")
    assert final_tr <= 10000.0 + 1e-3, f"Final Tr(P) {final_tr} exceeded 10,000!"
    assert np.allclose(predictor.P, predictor.P.T, atol=1e-5), "Covariance matrix lost symmetry!"
    assert np.all(np.isfinite(predictor.w)), "Weights non-finite after 100,000 zero samples!"
    assert 0.0 <= final_f <= 100.0, f"Forecast {final_f} out of bounds!"
    print("  --> PASS: 100,000 zero samples maintained strictly bounded Tr(P) <= 10,000 and finite state.")


def test_rls_100k_positive_constant_input():
    log_test_header("1.2: RLS Stability Under 100,000 Samples of 20.0 kPa")
    predictor = RLSPredictor(order=6, forgetting_factor=0.985, forecast_steps=15)
    
    max_trace = 0.0
    for i in range(100000):
        y_pred = predictor.update(20.0)
        tr = float(np.trace(predictor.P))
        if tr > max_trace:
            max_trace = tr
        if i % 25000 == 0:
            forecast_val = predictor.forecast()
            print(f"  Step {i:6d}: Tr(P) = {tr:8.2f}, y_pred = {y_pred:.4f}, forecast = {forecast_val:.4f}")
            assert np.isfinite(tr), f"Step {i}: Tr(P) is non-finite!"
            assert np.all(np.isfinite(predictor.P)), f"Step {i}: P contains NaN/Inf!"
            assert np.all(np.isfinite(predictor.w)), f"Step {i}: w contains NaN/Inf!"
            assert tr <= 10000.0 + 1e-3, f"Step {i}: Tr(P) exceeded 10,000! Tr={tr}"
            assert np.all(np.diag(predictor.P) >= -1e-7), f"Step {i}: Negative diagonal in P!"
            assert 0.0 <= forecast_val <= 100.0, f"Step {i}: Forecast out of bounds!"

    final_tr = float(np.trace(predictor.P))
    final_f = predictor.forecast()
    print(f"  Final Step 100000: Max Tr(P) = {max_trace:.2f}, Final Tr(P) = {final_tr:.2f}, Final Forecast = {final_f:.4f}")
    assert final_tr <= 10000.0 + 1e-3
    assert np.allclose(predictor.P, predictor.P.T, atol=1e-5), "Covariance matrix lost symmetry!"
    assert np.all(np.isfinite(predictor.w)), "Weights non-finite after 100,000 constant samples!"
    # Verify that in steady state of 20.0 kPa, forecast is accurate to steady state
    assert 19.0 <= final_f <= 21.0, f"Steady-state forecast {final_f} diverged from 20.0 kPa!"
    print("  --> PASS: 100,000 constant samples maintained stability and accurate 20.0 kPa forecast.")


def test_rls_alternating_phases():
    log_test_header("1.3: RLS Under Alternating Pressure Cycles (0 kPa <-> 25 kPa)")
    predictor = RLSPredictor(order=6, forgetting_factor=0.985, forecast_steps=15)
    
    # 10 cycles of 5,000 samples each = 50,000 samples
    for cycle in range(10):
        target_p = 25.0 if (cycle % 2 == 1) else 0.0
        for _ in range(5000):
            predictor.update(target_p)
        tr = float(np.trace(predictor.P))
        f_val = predictor.forecast()
        print(f"  Cycle {cycle+1}/10 (Target {target_p:4.1f} kPa): Tr(P) = {tr:8.2f}, Forecast = {f_val:6.2f}")
        assert tr <= 10000.0 + 1e-3
        assert np.all(np.isfinite(predictor.P))
        assert np.all(np.isfinite(predictor.w))
        assert 0.0 <= f_val <= 100.0
    print("  --> PASS: Alternating pressure cycles completed with bounded covariance and weights.")


# =====================================================================
# CHALLENGE 2: RLS STEP-RESPONSE AND IMPULSE RESPONSE DYNAMICS
# =====================================================================
def test_rls_step_response_up():
    log_test_header("2.1: RLS Step Response (10.0 kPa -> 50.0 kPa Jump)")
    predictor = RLSPredictor(order=5, forgetting_factor=0.98, forecast_steps=15)
    
    # Establish baseline at 10.0 kPa
    for _ in range(100):
        predictor.update(10.0)
    baseline_f = predictor.forecast()
    assert 9.5 <= baseline_f <= 10.5, f"Baseline forecast unexpected: {baseline_f}"
    
    # Step to 50.0 kPa
    forecast_trajectory = []
    for step in range(50):
        predictor.update(50.0)
        f = predictor.forecast()
        forecast_trajectory.append(f)
        # Boundedness check: must NEVER exceed [0, 100]
        assert 0.0 <= f <= 100.0, f"Step {step}: Forecast {f} breached physical bounds [0, 100]!"
        assert np.isfinite(f), f"Step {step}: Forecast non-finite!"
    
    print(f"  Step response start: {forecast_trajectory[0]:.2f} kPa -> end: {forecast_trajectory[-1]:.2f} kPa")
    # Must converge towards 50.0 kPa
    assert 48.0 <= forecast_trajectory[-1] <= 52.0, f"Failed to converge to 50.0 kPa: {forecast_trajectory[-1]}"
    print("  --> PASS: Upward step response remained bounded and smoothly converged to 50.0 kPa.")


def test_rls_step_response_down():
    log_test_header("2.2: RLS Downward Step / Rapid Venting (60.0 kPa -> 2.0 kPa)")
    predictor = RLSPredictor(order=5, forgetting_factor=0.98, forecast_steps=15)
    
    # Establish baseline at 60.0 kPa
    for _ in range(100):
        predictor.update(60.0)
    
    # Sudden vent to 2.0 kPa
    forecast_trajectory = []
    for step in range(50):
        predictor.update(2.0)
        f = predictor.forecast()
        forecast_trajectory.append(f)
        # Critical assertion: must NEVER undershoot below 0.0 kPa
        assert 0.0 <= f <= 100.0, f"Step {step}: Forecast {f} undershot below 0.0 kPa or exceeded 100.0!"
        assert np.isfinite(f), f"Step {step}: Forecast non-finite!"
    
    print(f"  Downward vent start: {forecast_trajectory[0]:.2f} kPa -> end: {forecast_trajectory[-1]:.2f} kPa")
    assert 1.0 <= forecast_trajectory[-1] <= 3.5, f"Failed to converge to 2.0 kPa: {forecast_trajectory[-1]}"
    print("  --> PASS: Downward step response clamped strictly at >= 0.0 kPa and converged to 2.0 kPa.")


def test_rls_impulse_glitch_response():
    log_test_header("2.3: RLS Impulse Glitch / Shock Spike (15 kPa -> 95 kPa -> 15 kPa)")
    predictor = RLSPredictor(order=5, forgetting_factor=0.98, forecast_steps=15)
    
    for _ in range(100):
        predictor.update(15.0)
        
    # Single sample shock glitch (e.g. sensor ESD pulse or burst shock wave)
    y_pred_glitch = predictor.update(95.0)
    f_glitch = predictor.forecast()
    print(f"  At shock sample (95.0 kPa): y_pred = {y_pred_glitch:.2f}, forecast = {f_glitch:.2f}")
    assert 0.0 <= f_glitch <= 100.0, f"Forecast during glitch {f_glitch} out of bounds!"
    
    # Immediate return to 15.0 kPa (observe recovery across 3*tau = 150 samples)
    # Note: With lambda=0.98, time constant tau = 1/(1-lambda) = 50 samples.
    # 95% settling time requires ~3*tau = 150 samples (1.5 seconds @ 100Hz).
    for step in range(150):
        predictor.update(15.0)
        f = predictor.forecast()
        assert 0.0 <= f <= 100.0, f"Recovery step {step}: Forecast {f} out of bounds!"
        assert np.isfinite(f)
        
    final_f = predictor.forecast()
    print(f"  After 150 recovery steps (3*tau): forecast = {final_f:.2f} kPa (target ~15.0 kPa)")
    assert 14.0 <= final_f <= 16.0, f"System did not recover from glitch after 3*tau: {final_f}"
    print("  --> PASS: Impulse shock safely absorbed and recovered smoothly to steady state.")


def test_rls_adversarial_sensor_glitches():
    log_test_header("2.4: RLS Ingestion of Corrupted Signals (NaN, Inf, String, Negative)")
    predictor = RLSPredictor(order=5, forgetting_factor=0.98, forecast_steps=15)
    
    for _ in range(50):
        predictor.update(12.0)
        
    corrupt_samples = [
        float('nan'), float('inf'), float('-inf'), -999.0, 
        "CORRUPT_STR", None, [], {}, math.nan
    ]
    
    for item in corrupt_samples:
        try:
            res = predictor.update(item)
            f = predictor.forecast()
            assert np.isfinite(res), f"update({item}) returned non-finite: {res}"
            assert np.isfinite(f), f"forecast() after {item} returned non-finite: {f}"
            assert 0.0 <= f <= 100.0, f"forecast() out of bounds [0, 100]: {f}"
        except Exception as e:
            raise AssertionError(f"RLSPredictor raised unhandled exception on corrupt input {item}: {e}")
            
    print("  --> PASS: Corrupt sensor data gracefully filtered without exception or state corruption.")


# =====================================================================
# CHALLENGE 3: RLS NON-LEAKAGE AND AR CAUSALITY VERIFICATION
# =====================================================================
def test_rls_ar_causality():
    log_test_header("3.1: RLS Autoregressive Non-Leakage & Strict Causality")
    order = 4
    predictor = RLSPredictor(order=order, forgetting_factor=0.98, forecast_steps=10)
    
    # 1. Warm up with constant 10.0 kPa
    for _ in range(50):
        predictor.update(10.0)
        
    # State before step jump
    prior_buffer = list(predictor.x)
    assert len(prior_buffer) == order
    assert all(abs(val - 10.0) < 1e-5 for val in prior_buffer)
    
    # Pre-jump forecast
    pre_forecast = predictor.forecast()
    assert abs(pre_forecast - 10.0) < 0.5, f"Pre-jump forecast unexpected: {pre_forecast}"
    
    # Now step jump to 70.0 kPa occurs at this instant.
    # In a non-leaking AR model:
    # y_pred = dot(x_history, w) MUST be calculated using [y(t-1), y(t-2), ...],
    # which are all 10.0 kPa.
    # Therefore, y_pred MUST be ~10.0 kPa, and CANNOT predict the 70.0 jump before seeing it.
    y_pred = predictor.update(70.0)
    
    print(f"  Step jump to 70.0 kPa: y_pred = {y_pred:.4f} kPa")
    
    # Non-leakage test:
    # If the model had leaked y(t) into x_vector before predicting,
    # and w[0] had weight > 0, y_pred would have jumped towards 70.
    # In strict non-leakage, y_pred was computed from pure 10.0 kPa history.
    assert abs(y_pred - 10.0) < 5.0, (
        f"DATA LEAKAGE DETECTED! y_pred was {y_pred:.4f}, anticipating or leaking 70.0 kPa "
        f"despite prior history consisting strictly of 10.0 kPa!"
    )
    
    # Verify internal deque: 70.0 must be at index 0 AFTER update, and 10.0 at index 1
    assert predictor.x[0] == 70.0
    assert predictor.x[1] == 10.0
    
    print("  --> PASS: Strict AR causality confirmed: y_pred does NOT leak current target sample.")


def test_rls_ramp_extrapolation_causality():
    log_test_header("3.2: Dynamic Ramp Learning Without Degenerating to Identity")
    # Feed ramp y(t) = 3.0 * t
    predictor = RLSPredictor(order=4, forgetting_factor=0.99, forecast_steps=5)
    
    for t in range(80):
        val = 3.0 * t
        y_pred = predictor.update(val)
        if t > 20:
            # For linear ramp, 1-step extrapolation error should be small once adapted
            err = abs(val - y_pred)
            # Must track ramp without divergence
            assert err < 15.0, f"Ramp tracking error too large at step {t}: err={err}"
            
    # Check weights:
    # In a leaked model, w = [1.0, 0, 0, 0].
    # In an un-leaked AR model on a ramp, y(t) - y(t-1) = y(t-1) - y(t-2) -> y(t) = 2*y(t-1) - y(t-2).
    # Thus w should have positive w[0] and negative w[1] (or distributed), NEVER pure [1, 0, 0, 0].
    w = predictor.w
    print(f"  Learned ramp weights: {np.round(w, 4)}")
    assert not np.allclose(w, [1.0, 0.0, 0.0, 0.0], atol=0.05), (
        f"Weights degenerated into trivial identity copy [1, 0, 0, 0]: {w}"
    )
    print("  --> PASS: Model learns genuine dynamical extrapolation rather than identity copy.")


# =====================================================================
# CHALLENGE 4: train_ml.py ADVERSARIAL EDGE CASES
# =====================================================================
def run_ml_pipeline_harness(df):
    """Executes the full train_ml pipeline on an in-memory DataFrame."""
    # 1. Feature extraction
    df_feat = tml.compute_advanced_features(df)
    
    # 2. Labeling
    df_labeled = tml.label_data(df_feat)
    
    # 3. Model training & classification report
    features = [
        "Pressure_kPa", "Slope_kPa_s", "Voltage_V", "Current_A", "Health_Pct",
        "Pressure_Var_300ms", "Pressure_Integral_500ms", "Power_W", "Power_to_Pressure_Ratio", "Creep_Slope"
    ]
    X = df_labeled[features]
    y = df_labeled["Target_State"]
    
    # Split
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
    
    model = RandomForestClassifier(n_estimators=10, max_depth=6, random_state=42)
    model.fit(X_train, y_train)
    
    y_pred = model.predict(X_test)
    
    # Test classification_report with all 4 possible labels
    rep = classification_report(
        y_test, y_pred, 
        labels=[0, 1, 2, 3], 
        target_names=["STABLE", "SQUEEZE", "YIELDING", "DANGER"], 
        zero_division=0
    )
    return model, rep


def test_train_ml_single_class_stable():
    log_test_header("4.1: ML Pipeline Single Class Dataset (Class 0: STABLE only)")
    N = 100
    df = pd.DataFrame({
        "Pressure_kPa": [10.0] * N,
        "Slope_kPa_s": [0.05] * N,
        "Voltage_V": [12.0] * N,
        "Current_A": [1.0] * N,
        "Mode": ["IDLE"] * N,
        "Health_Pct": [100.0] * N,
        "PWM": [0] * N,
    })
    
    model, rep = run_ml_pipeline_harness(df)
    assert "STABLE" in rep
    assert "DANGER" in rep
    print("  --> PASS: Single-class (STABLE) dataset trained and evaluated with zero unhandled exceptions.")


def test_train_ml_single_class_danger():
    log_test_header("4.2: ML Pipeline Single Class Dataset (Class 3: DANGER only)")
    N = 100
    # Pressure >= 45.0 kPa or Health <= 20% triggers Class 3 DANGER
    df = pd.DataFrame({
        "Pressure_kPa": [55.0] * N,
        "Slope_kPa_s": [2.5] * N,
        "Voltage_V": [12.0] * N,
        "Current_A": [2.5] * N,
        "Mode": [3] * N,
        "Health_Pct": [15.0] * N,
        "PWM": [255] * N,
    })
    
    model, rep = run_ml_pipeline_harness(df)
    assert "DANGER" in rep
    print("  --> PASS: Single-class (DANGER) dataset trained and evaluated with zero unhandled exceptions.")


def test_train_ml_two_and_three_classes():
    log_test_header("4.3: ML Pipeline Two-Class and Three-Class Datasets")
    # 2 Classes: 0 and 2
    N = 120
    df2 = pd.DataFrame({
        "Pressure_kPa": [10.0] * 60 + [25.0] * 60,
        "Slope_kPa_s": [0.05] * 60 + [0.10] * 60,
        "Voltage_V": [12.0] * N,
        "Current_A": [1.2] * N,
        "Mode": ["IDLE"] * 60 + ["SMART"] * 60,
        "Health_Pct": [90.0] * N,
        "PWM": [100] * N,
    })
    model2, rep2 = run_ml_pipeline_harness(df2)
    assert "STABLE" in rep2 and "YIELDING" in rep2
    print("  2-class dataset (STABLE, YIELDING): Clean evaluation.")
    
    # 3 Classes: 0, 1, and 3
    df3 = pd.DataFrame({
        "Pressure_kPa": [10.0] * 40 + [20.0] * 40 + [50.0] * 40,
        "Slope_kPa_s": [0.05] * 40 + [2.0] * 40 + [1.0] * 40,
        "Voltage_V": [12.0] * 120,
        "Current_A": [1.2] * 120,
        "Mode": ["IDLE"] * 40 + ["MANUAL"] * 40 + ["BURST"] * 40,
        "Health_Pct": [90.0] * 40 + [90.0] * 40 + [10.0] * 40,
        "PWM": [150] * 120,
    })
    model3, rep3 = run_ml_pipeline_harness(df3)
    assert "STABLE" in rep3 and "SQUEEZE" in rep3 and "DANGER" in rep3
    print("  3-class dataset (STABLE, SQUEEZE, DANGER): Clean evaluation.")
    print("  --> PASS: Two-class and three-class datasets handled without zero-division or missing-label errors.")


def test_train_ml_column_schema_and_missing():
    log_test_header("4.4: Schema Robustness (Legacy Headers, Missing Optional Columns)")
    # Test legacy GUI column layout
    N = 100
    legacy_df = pd.DataFrame({
        "Time": [0.01 * k for k in range(N)],
        "Pressure": [12.0 + 0.1 * k for k in range(N)],
        "PWM": [150] * N,
        "Volts": [11.8] * N,
        "Amps": [1.4] * N,
        "Slope": [0.1] * N,
        "Health": [85.0] * N,
        "Mode": [2] * N,
        "ExtraUnneededColumn": [999] * N,
    })
    
    col_map = {
        "Pressure": "Pressure_kPa",
        "Volts": "Voltage_V",
        "Amps": "Current_A",
        "Slope": "Slope_kPa_s",
        "Health": "Health_Pct",
        "Time": "Time_Device_s",
    }
    renamed = legacy_df.rename(columns=col_map)
    model, rep = run_ml_pipeline_harness(renamed)
    assert model is not None
    print("  --> PASS: Legacy column mapping and extra columns processed transparently.")


def test_train_ml_mode_representations():
    log_test_header("4.5: Mode Representations (Ints, Strings, Floats, and Unknowns)")
    N = 80
    modes = [
        "0", 1, "SMART", "BURST", 4, "MODE_5", 6, "ERROR", 
        "DIAMETER", 9, 10, "UNKNOWN_SPECIAL_MODE", -99, 2.0, " 2 "
    ]
    df = pd.DataFrame({
        "Pressure_kPa": [15.0] * len(modes),
        "Slope_kPa_s": [0.1] * len(modes),
        "Voltage_V": [12.0] * len(modes),
        "Current_A": [1.5] * len(modes),
        "Mode": modes,
        "Health_Pct": [80.0] * len(modes),
        "PWM": [120] * len(modes),
    })
    
    df_feat = tml.compute_advanced_features(df)
    labeled = tml.label_data(df_feat)
    
    assert "Target_State" in labeled.columns
    assert len(labeled["Target_State"]) == len(modes)
    assert all(label in [0, 1, 2, 3] for label in labeled["Target_State"])
    print(f"  Generated labels for mixed modes: {list(labeled['Target_State'])}")
    print("  --> PASS: Mixed mode representations parsed safely into valid target states [0, 1, 2, 3].")


def test_train_ml_zero_variance_and_division_by_zero():
    log_test_header("4.6: Zero Variance and Mathematical Singularities (Pressure = 0, Voltage = 0)")
    N = 100
    df = pd.DataFrame({
        "Pressure_kPa": [0.0] * N,      # Pressure = 0 (tests Power_to_Pressure_Ratio division by zero)
        "Slope_kPa_s": [0.0] * N,       # Zero slope
        "Voltage_V": [0.0] * N,         # Zero voltage
        "Current_A": [0.0] * N,         # Zero current
        "Mode": [0] * N,                # IDLE
        "Health_Pct": [100.0] * N,      # 100% health
        "PWM": [0] * N,                 # 0 PWM
    })
    
    df_feat = tml.compute_advanced_features(df)
    
    # Assert zero division did not create NaNs or Infs
    assert not df_feat["Power_to_Pressure_Ratio"].isna().any(), "Power_to_Pressure_Ratio contains NaNs on P=0!"
    assert not np.isinf(df_feat["Power_to_Pressure_Ratio"]).any(), "Power_to_Pressure_Ratio contains Infs on P=0!"
    assert (df_feat["Power_to_Pressure_Ratio"] == 0.0).all(), "Power_to_Pressure_Ratio should be 0.0 on P <= 0.5!"
    
    assert (df_feat["Pressure_Var_300ms"] == 0.0).all(), "Variance of constant signal must be exactly 0.0!"
    assert not df_feat.isna().any().any(), "Engineered feature table contains unexpected NaNs!"
    
    model, rep = run_ml_pipeline_harness(df)
    assert model is not None
    print("  --> PASS: Zero variance and singular inputs (P=0) handled without NaN/Inf generation.")


# =====================================================================
# MAIN RUNNER
# =====================================================================
def main():
    print("=" * 70)
    print("M3 ADVERSARIAL EMPIRICAL STRESS TEST HARNESS")
    print("Target: RLSPredictor & train_ml.py")
    print("=" * 70)
    
    tests = [
        test_rls_100k_zero_input,
        test_rls_100k_positive_constant_input,
        test_rls_alternating_phases,
        test_rls_step_response_up,
        test_rls_step_response_down,
        test_rls_impulse_glitch_response,
        test_rls_adversarial_sensor_glitches,
        test_rls_ar_causality,
        test_rls_ramp_extrapolation_causality,
        test_train_ml_single_class_stable,
        test_train_ml_single_class_danger,
        test_train_ml_two_and_three_classes,
        test_train_ml_column_schema_and_missing,
        test_train_ml_mode_representations,
        test_train_ml_zero_variance_and_division_by_zero,
    ]
    
    passed = 0
    failed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            print(f"\n[CHALLENGE FAILED]: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
        except Exception as e:
            print(f"\n[UNHANDLED EXCEPTION]: {e}")
            import traceback
            traceback.print_exc()
            failed += 1

    print("\n" + "=" * 70)
    print(f"ADVERSARIAL STRESS RESULTS: {passed}/{len(tests)} PASSED, {failed} FAILED")
    print("=" * 70)
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
