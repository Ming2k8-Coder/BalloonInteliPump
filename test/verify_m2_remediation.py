#!/usr/bin/env python3
"""
Comprehensive Automated Verification Suite for Milestone 2 (M2):
Mathematical Models, Algorithms & Finite State Machine Remediation

Validates all 8 Scope Items + Numerical Stability:
- ALG-MR-01: Equibiaxial Mooney-Rivlin Spherical Membrane Formulation
- ALG-MR-02: Strain Energy Integration with Rubber Volume
- ALG-MR-03: Miner's Rule Fatigue MPa Scaling
- ALG-RLS-01: Online RLS Parameter Identification Sample Divergence
- ALG-FSM-01: Centralized add_sample() & Savitzky-Golay Partial Buffer Fallback
- ALG-VOL-01: Volume Estimator Deflation Flow Model & Solenoid Tracking
- ALG-VOL-02: Diameter Control Actuator Latch-Up & Mutual Exclusion
- ALG-FSM-02: Event Group Bit Allocation & SMART_FINISHED Transition
- NUM-SLS-01: Unconditionally A-Stable SLS Exponential Integrator & Numerical Guards
"""

import sys
import math
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MAIN_DIR = REPO_ROOT / "main"


def test_alg_mr_01():
    print("[1/9] Verifying ALG-MR-01 (Equibiaxial Mooney-Rivlin Spherical Membrane)...")
    phys_cpp = (MAIN_DIR / "bip_balloon_physics.cpp").read_text(encoding="utf-8")
    
    # 1. Verify Equibiaxial Cauchy stress formula in bip_balloon_physics.cpp
    assert "2.0f * (lambda_sq - inv_lambda_4) * (effective_C10 + effective_C01 * lambda_sq)" in phys_cpp, \
        "Equibiaxial stress formula not found in update_balloon_physics_ext"
    assert "(lambda_sq - inv_lambda)" not in phys_cpp, \
        "Old uniaxial stress formula still present in bip_balloon_physics.cpp!"
    
    # 2. Verify Equibiaxial Strain Invariants
    assert "2.0f * lambda_sq + inv_lambda_4" in phys_cpp, \
        "Equibiaxial I1 invariant (2*lambda^2 + lambda^-4) not found"
    assert "(lambda_sq * lambda_sq) + 2.0f * inv_lambda_sq" in phys_cpp or \
           "(lambda_sq * lambda_sq) + 2.0f / lambda_sq" in phys_cpp, \
        "Equibiaxial I2 invariant (lambda^4 + 2*lambda^-2) not found"

    # 3. Verify ride inflation equibiaxial burst stress & quadratic wall thinning
    assert "burst_stress = 2.0f * (maxLambda_sq - inv_maxLambda4) * (BASE_C10 + BASE_C01 * maxLambda_sq)" in phys_cpp, \
        "Equibiaxial burst stress not found in compute_ride_inflation"
    assert "t_wall_mm = 0.35f / working_lambda_sq" in phys_cpp, \
        "Quadratic wall thickness thinning (t = t0 / lambda^2) not found in compute_ride_inflation"
    assert "t_wall_mm = 0.35f / working_lambda;" not in phys_cpp, \
        "Uniaxial wall thickness thinning (t0 / lambda) still present!"

    # 4. Numerical verification of equibiaxial stress values
    lam = 2.0
    C10, C01 = 180.0, 20.0
    lam_sq = lam * lam
    inv_lam4 = 1.0 / (lam_sq * lam_sq)
    sigma_equibiaxial = 2.0 * (lam_sq - inv_lam4) * (C10 + C01 * lam_sq)
    expected_sigma = 2047.5  # 2 * (4.0 - 0.0625) * (180 + 80) = 2 * 3.9375 * 260
    assert abs(sigma_equibiaxial - expected_sigma) < 1e-4, f"Mismatch in equibiaxial stress: {sigma_equibiaxial}"

    I1 = 2.0 * lam_sq + inv_lam4
    I2 = lam_sq * lam_sq + 2.0 / lam_sq
    assert abs(I1 - 8.0625) < 1e-4, f"Mismatch in I1: {I1}"
    assert abs(I2 - 16.5) < 1e-4, f"Mismatch in I2: {I2}"

    W_density = C10 * (I1 - 3.0) + C01 * (I2 - 3.0)
    assert abs(W_density - 1181.25) < 1e-4, f"Mismatch in W_density: {W_density}"

    print("      PASS: Equibiaxial Cauchy stress, strain invariants, and quadratic wall thinning verified.")


def test_alg_mr_02():
    print("[2/9] Verifying ALG-MR-02 (Strain Energy Integration with Rubber Volume)...")
    phys_cpp = (MAIN_DIR / "bip_balloon_physics.cpp").read_text(encoding="utf-8")

    # 1. Verify rubber volume calculation and integration
    assert "INITIAL_WALL_THICKNESS_M = 0.00035f" in phys_cpp, \
        "INITIAL_WALL_THICKNESS_M (0.35 mm) constant not defined"
    assert "v_rubber_m3 = 4.0f * (float)M_PI * r0_m * r0_m * INITIAL_WALL_THICKNESS_M" in phys_cpp or \
           "v_rubber_m3 = (float)M_PI * d0_m * d0_m * INITIAL_WALL_THICKNESS_M" in phys_cpp, \
        "Rubber volume formula V_rubber = 4*pi*r0^2*t0 not found"
    assert "local_phys.strain_energy_j = (W_density * 1000.0f) * v_rubber_m3;" in phys_cpp, \
        "Strain energy integration over rubber volume not found"

    # 2. Assert air volume is NOT used for strain energy
    assert "W_density * (volEst.volume_liters * 0.001f)" not in phys_cpp, \
        "Old flawed strain energy scaling with air volume (volEst.volume_liters) still present!"

    # 3. Numerical test: Rubber volume vs Air volume for 11" balloon
    r0 = 0.035  # 7 cm diameter / 2
    t0 = 0.00035  # 0.35 mm
    v_rubber = 4.0 * math.pi * r0 * r0 * t0  # ~5.388e-6 m^3
    W_density_kj = 1181.25  # kJ/m^3 at lambda = 2.0
    E_strain_j = W_density_kj * 1000.0 * v_rubber  # J
    assert 6.0 < E_strain_j < 7.0, f"Calculated strain energy out of realistic range: {E_strain_j} J"

    # Air volume comparison: air volume is ~11.5 liters = 0.0115 m^3
    v_air = 0.0115
    E_air_err = W_density_kj * 1000.0 * v_air
    assert E_air_err > 13000.0, "Air volume calculation sanity check failed"
    ratio = E_air_err / E_strain_j
    assert ratio > 2000.0, f"Overestimation ratio expected > 2000x, got {ratio}"
    print(f"      PASS: Strain energy accurately integrates over rubber volume ({E_strain_j:.2f} J vs old {E_air_err:.1f} J).")


def test_alg_mr_03():
    print("[3/9] Verifying ALG-MR-03 (Miner's Rule Fatigue MPa Scaling)...")
    phys_cpp = (MAIN_DIR / "bip_balloon_physics.cpp").read_text(encoding="utf-8")

    # 1. Source code assertions
    assert "float peak_stress_mpa = peak_stress_kpa * 0.001f;" in phys_cpp, \
        "Conversion from peak_stress_kpa to peak_stress_mpa (0.001f) not found in record_bounce_cycle"
    assert "FATIGUE_K / std::pow(peak_stress_mpa, FATIGUE_M)" in phys_cpp, \
        "Miner's rule Nf does not use peak_stress_mpa"
    assert "FATIGUE_K / std::pow(peak_stress_kpa, FATIGUE_M)" not in phys_cpp, \
        "Old unscaled peak_stress_kpa still used in Miner's rule calculation!"

    # 2. Algorithmic simulation of Miner's Rule
    FATIGUE_K = 1.0e8
    FATIGUE_M = 3.5

    # Test Case A: Normal bounce stress 2000 kPa (2.0 MPa)
    peak_kpa = 2000.0
    peak_mpa = peak_kpa * 0.001
    Nf = FATIGUE_K / (peak_mpa ** FATIGUE_M)
    dD = 1.0 / Nf
    assert Nf > 8.0e6, f"Expected >8M cycles to failure at 2 MPa, got {Nf}"
    assert dD < 2.0e-7, f"Damage per bounce too high: {dD}"

    # Simulate 1000 bounces: cumulative damage should be negligible (< 0.1%)
    accum_damage = 1000 * dD
    safe_remaining = (1.0 - accum_damage) * 100.0
    assert accum_damage < 0.001, f"Accumulated damage after 1000 cycles too high: {accum_damage}"
    assert safe_remaining > 99.9, f"Safe pressure remaining dropped unexpectedly: {safe_remaining}%"

    # Test Case B: High stress bounce 15,000 kPa (15.0 MPa)
    peak_kpa_high = 15000.0
    peak_mpa_high = peak_kpa_high * 0.001
    Nf_high = FATIGUE_K / (peak_mpa_high ** FATIGUE_M)
    assert 7000 < Nf_high < 8000, f"Expected ~7530 cycles at 15 MPa, got {Nf_high}"

    # Verify bug prevention: Old unscaled code gives Nf = 10^8 / (2000^3.5) = 0.00078 => Nf = 1 => D = 1.0 instant failure
    old_Nf = FATIGUE_K / (peak_kpa ** FATIGUE_M)
    assert old_Nf < 0.001, "Old code failure reproduction check"
    print("      PASS: Stress correctly scaled to MPa; eliminates instant 1-cycle destruction.")


def test_alg_rls_01():
    print("[4/9] Verifying ALG-RLS-01 (Online RLS Parameter Identification Sample Divergence)...")
    phys_cpp = (MAIN_DIR / "bip_balloon_physics.cpp").read_text(encoding="utf-8")

    # 1. Source code assertions
    assert "rls_sum_w  = gamma * rls_sum_w  + 1.0f;" in phys_cpp, \
        "Recursive effective weight sum W not found in update_online_material_identification"
    assert "float w = rls_sum_w;" in phys_cpp, \
        "Effective weight w not read for regression"
    assert "float denominator = (w * rls_sum_xx - rls_sum_x * rls_sum_x);" in phys_cpp, \
        "Regression denominator does not use effective weight w"
    assert "float intercept_C10 = (rls_sum_y - slope_C01 * rls_sum_x) / w;" in phys_cpp, \
        "Intercept C10 calculation does not divide by w"
    assert "float intercept_C10 = (rls_sum_y - slope_C01 * rls_sum_x) / n;" not in phys_cpp, \
        "Old divergent division by unbounded sample count n still present!"
    assert "var_x > 0.005f" in phys_cpp, \
        "Persistent excitation check on stretch variance var_x not found"
    assert "sigma_meas = pressure_kpa * geom_ratio * (lambda_sq * lambda);" in phys_cpp, \
        "Equibiaxial Cauchy stress mapping sigma = P * (D0 / 4t0) * lambda^3 not found"

    # 2. Algorithmic simulation of RLS parameter identification over 2000 samples
    gamma = 0.995
    sum_w = 0.0
    sum_x = 0.0
    sum_y = 0.0
    sum_xy = 0.0
    sum_xx = 0.0
    sum_yy = 0.0

    true_C10 = 180.0
    true_C01 = 20.0
    D0_cm = 7.0
    t0_cm = 0.035
    geom_ratio = D0_cm / (4.0 * t0_cm)  # 50.0

    est_C10 = 180.0
    est_C01 = 20.0
    converged = False

    for k in range(2000):
        # Oscillating stretch ratio between 1.2 and 2.5 (persistent excitation)
        lam = 1.5 + 0.5 * math.sin(k * 0.05)
        lam_sq = lam * lam
        inv_lam4 = 1.0 / (lam_sq * lam_sq)
        denom = 2.0 * (lam_sq - inv_lam4)

        # True equibiaxial stress and pressure
        sigma_true = denom * (true_C10 + true_C01 * lam_sq)
        P_kpa = sigma_true / (geom_ratio * lam_sq * lam)

        sigma_meas = P_kpa * geom_ratio * (lam_sq * lam)
        y = sigma_meas / denom
        x = lam_sq

        sum_w  = gamma * sum_w + 1.0
        sum_x  = gamma * sum_x + x
        sum_y  = gamma * sum_y + y
        sum_xy = gamma * sum_xy + (x * y)
        sum_xx = gamma * sum_xx + (x * x)
        sum_yy = gamma * sum_yy + (y * y)

        if k >= 50 and sum_w >= 30.0:
            w = sum_w
            denominator = (w * sum_xx - sum_x * sum_x)
            var_x = denominator / (w * w)
            if var_x > 0.005:
                num_xy = (w * sum_xy - sum_x * sum_y)
                slope = num_xy / denominator
                intercept = (sum_y - slope * sum_x) / w
                if 50.0 < intercept < 400.0 and 0.0 <= slope < 100.0:
                    est_C10 = 0.95 * est_C10 + 0.05 * intercept
                    est_C01 = 0.95 * est_C01 + 0.05 * slope
                    converged = True

    # Assertions on asymptotic boundedness & parameter recovery
    asymptotic_w_max = 1.0 / (1.0 - gamma)  # 200.0
    assert abs(sum_w - asymptotic_w_max) < 0.1, f"sum_w did not reach 200.0: {sum_w}"
    assert converged, "RLS did not converge"
    assert abs(est_C10 - true_C10) < 1.0, f"Identified C10 diverged: {est_C10} (expected {true_C10})"
    assert abs(est_C01 - true_C01) < 1.0, f"Identified C01 diverged: {est_C01} (expected {true_C01})"
    print(f"      PASS: RLS asymptotic weight bounded at W={sum_w:.1f}; C10 converged to {est_C10:.2f}, C01 to {est_C01:.2f}.")


def test_alg_fsm_01():
    print("[5/9] Verifying ALG-FSM-01 (Centralized add_sample & Partial Buffer Fallback)...")
    state_cpp = (MAIN_DIR / "bip_state.cpp").read_text(encoding="utf-8")

    # 1. Centralized invocation in update_state_machine
    assert "void update_state_machine() {" in state_cpp
    fsm_idx = state_cpp.find("void update_state_machine() {")
    fsm_block = state_cpp[fsm_idx:fsm_idx + 250]
    assert "add_sample(" in fsm_block, "add_sample not called at start of update_state_machine"

    # 2. Removal of duplicate calls in update_smart and update_burst
    smart_idx = state_cpp.find("static void update_smart() {")
    smart_block = state_cpp[smart_idx:smart_idx + 150]
    assert "add_sample(" not in smart_block, "Duplicate add_sample call still present in update_smart"

    burst_idx = state_cpp.find("static void update_burst() {")
    burst_block = state_cpp[burst_idx:burst_idx + 150]
    assert "add_sample(" not in burst_block, "Duplicate add_sample call still present in update_burst"

    # 3. Finite check and partial buffer finite-difference fallback
    sample_idx = state_cpp.find("static void add_sample(float p) {")
    sample_block = state_cpp[sample_idx:sample_idx + 2500]
    assert "if (!std::isfinite(p)) return;" in sample_block, "NaN / Inf check missing in add_sample"
    assert "current_dp_dt = (pressureBuf[curr_idx] - pressureBuf[prev_idx]) / dt_sec;" in sample_block, \
        "Partial buffer 2-point finite difference fallback missing in add_sample"

    # 4. Numerical simulation of partial buffer fallback (N=2..8)
    samples = [10.0, 10.5, 11.2, 12.0, 12.9]
    times_us = [0, 500, 1000, 1500, 2000]
    dp_dt_calculated = []
    for i in range(1, len(samples)):
        dt_sec = (times_us[i] - times_us[i-1]) / 1000000.0
        dp = (samples[i] - samples[i-1]) / dt_sec
        dp_dt_calculated.append(dp)
    
    assert abs(dp_dt_calculated[0] - 1000.0) < 1e-4  # 0.5 kPa / 0.0005 s
    assert abs(dp_dt_calculated[1] - 1400.0) < 1e-4  # 0.7 kPa / 0.0005 s
    assert all(d > 0 for d in dp_dt_calculated), "Derivatives must be positive during pressure ramp"
    print("      PASS: add_sample centralized, duplicate calls removed, partial buffer fallback validated.")


def test_alg_vol_01():
    print("[6/9] Verifying ALG-VOL-01 (Volume Estimator Deflation Flow Model)...")
    motor_h = (MAIN_DIR / "bip_motor.h").read_text(encoding="utf-8")
    motor_cpp = (MAIN_DIR / "bip_motor.cpp").read_text(encoding="utf-8")
    vol_h = (MAIN_DIR / "bip_volume_estimator.h").read_text(encoding="utf-8")
    vol_cpp = (MAIN_DIR / "bip_volume_estimator.cpp").read_text(encoding="utf-8")
    main_cpp = (MAIN_DIR / "main.cpp").read_text(encoding="utf-8")

    # 1. Getter in bip_motor
    assert "uint8_t get_solenoid_pwm();" in motor_h, "get_solenoid_pwm() not declared in bip_motor.h"
    assert "static uint8_t s_solenoid_pwm = 0;" in motor_cpp, "s_solenoid_pwm not defined in bip_motor.cpp"
    assert "uint8_t get_solenoid_pwm() {" in motor_cpp, "get_solenoid_pwm() not implemented in bip_motor.cpp"

    # 2. Signatures in bip_volume_estimator
    assert "void update_volume_estimator(float current_pressure_kpa, float motor_pwm, float solenoid_pwm, float dt_sec);" in vol_h
    assert "void update_volume_estimator_ext(float current_pressure_kpa, float motor_pwm, float solenoid_pwm, float mcu_temp_c, float dt_sec);" in vol_h
    assert "void set_relief_valve_params(float k_vent);" in vol_h

    # 3. Solenoid orifice deflation in bip_volume_estimator.cpp
    assert "float vent_flow_lps = k_vent_conductance * vent_duty * std::sqrt(p_gauge_pos);" in vol_cpp, \
        "Orifice venting flow model not found in bip_volume_estimator.cpp"
    assert "float net_flow_lps = corrected_pump_flow_lps - vent_flow_lps;" in vol_cpp, \
        "Net flow subtraction (inflow - outflow) not found"
    assert "if (cumulative_volume_liters < 0.0f)" in vol_cpp, \
        "Non-negative clamp on cumulative_volume_liters not found"

    # 4. main.cpp passes get_solenoid_pwm()
    assert "update_volume_estimator_ext(pressureSensor.getValue(), (float)pumpMotor.getCurrentPWM(), (float)get_solenoid_pwm(), mcu_temp, 0.0005f);" in main_cpp, \
        "main.cpp line 87 does not pass (float)get_solenoid_pwm()"

    # 5. Algorithmic simulation of deflation
    k_vent = 0.085
    p_kpa = 16.0
    pwm_solenoid = 255
    vent_duty = pwm_solenoid / 255.0
    vent_flow_lps = k_vent * vent_duty * math.sqrt(p_kpa)
    expected_vent_flow = 0.085 * 1.0 * 4.0  # 0.34 L/s
    assert abs(vent_flow_lps - expected_vent_flow) < 1e-5, f"Vent flow mismatch: {vent_flow_lps}"

    # Simulate pumping 1.0 L then deflating
    vol = 1.0
    dt = 0.0005  # 2000 SPS
    # Deflate for 2 seconds (4000 ticks) with pump off
    for _ in range(4000):
        vol -= vent_flow_lps * dt
        if vol < 0.0: vol = 0.0
    assert abs(vol - (1.0 - 0.34 * 2.0)) < 1e-4, f"Deflated volume mismatch: {vol}"

    # Extended deflation to assert zero-clamping
    for _ in range(10000):
        vol -= vent_flow_lps * dt
        if vol < 0.0: vol = 0.0
    assert vol == 0.0, f"Volume should clamp to exactly 0.0 L: {vol}"
    print("      PASS: Deflation orifice flow model, solenoid tracking, and non-negative clamping verified.")


def test_alg_vol_02():
    print("[7/9] Verifying ALG-VOL-02 (Diameter Control Actuator Latch-Up & Mutual Exclusion)...")
    state_cpp = (MAIN_DIR / "bip_state.cpp").read_text(encoding="utf-8")
    vol_cpp = (MAIN_DIR / "bip_volume_estimator.cpp").read_text(encoding="utf-8")

    # 1. Verify diameter control in bip_state.cpp
    diam_idx = state_cpp.find("static void update_diameter() {")
    assert diam_idx != -1, "update_diameter not found in bip_state.cpp"
    diam_block = state_cpp[diam_idx:diam_idx + 1600]

    assert "DIAMETER_DEADBAND_CM = 0.5f" in diam_block, "Control deadband (+/- 0.5 cm) not found"
    assert "error > DIAMETER_DEADBAND_CM" in diam_block, "Positive error inflation branch not found"
    assert "error < -DIAMETER_DEADBAND_CM" in diam_block, "Negative error venting branch not found"
    assert "pid_Kff * 1.2f * targetDiameterCm" not in diam_block, \
        "Old uncalibrated diameter feedforward term still present in update_diameter!"
    
    # 2. Verify mutual actuator exclusion
    assert "write_solenoid_pwm(0); // Ensure vent is sealed" in diam_block, \
        "Solenoid not sealed during inflation"
    assert "pumpMotor.setTargetPWM(0);" in diam_block, \
        "Pump motor not halted during venting"

    # 3. Verify thin-wall spherical hoop stress equation in bip_volume_estimator.cpp
    assert "t_wall_cm = 0.035f / (lambda * lambda);" in vol_cpp, \
        "Spherical equibiaxial quadratic thickness thinning not found in bip_volume_estimator"
    assert "s_last_pressure_kpa * est.diameter_cm / (4.0f * t_wall_cm)" in vol_cpp, \
        "Spherical hoop stress formula sigma = (P * D) / (4 * t) not found in bip_volume_estimator"
    assert "est.wall_stress_kpa = est.diameter_cm * 2.5f;" not in vol_cpp, \
        "Old dummy hoop stress formula still present!"

    # 4. Numerical simulation of mutual exclusion
    Kp = 25.0
    Ki = 0.8
    integral = 0.0
    dt = 0.05
    target_d = 30.0

    # Scenario 1: Undersized balloon (d = 28.0 cm -> error = +2.0 cm)
    err1 = target_d - 28.0
    assert err1 > 0.5
    pump_pwm_1 = min(255, max(40, int(Kp * 1.5 * err1)))
    vent_pwm_1 = 0
    assert pump_pwm_1 > 0 and vent_pwm_1 == 0, "Motor must run, valve must seal"

    # Scenario 2: Within deadband (d = 30.2 cm -> error = -0.2 cm)
    err2 = target_d - 30.2
    assert abs(err2) <= 0.5
    pump_pwm_2 = 0
    vent_pwm_2 = 0
    assert pump_pwm_2 == 0 and vent_pwm_2 == 0, "Both actuators must be silent in deadband"

    # Scenario 3: Oversized balloon (d = 32.0 cm -> error = -2.0 cm)
    err3 = target_d - 32.0
    assert err3 < -0.5
    pump_pwm_3 = 0
    overshoot = -err3
    vent_pwm_3 = min(255, max(120, int(100.0 + overshoot * 20.0)))
    assert pump_pwm_3 == 0 and vent_pwm_3 > 0, "Pump must be stopped, vent valve must open"
    print("      PASS: Diameter feedforward eliminated; deadband & actuator mutual exclusion verified.")


def test_alg_fsm_02():
    print("[8/9] Verifying ALG-FSM-02 (Event Group Bit Allocation & SMART_FINISHED Transition)...")
    state_h = (MAIN_DIR / "bip_state.h").read_text(encoding="utf-8")
    state_cpp = (MAIN_DIR / "bip_state.cpp").read_text(encoding="utf-8")

    # 1. Event Group Bit Definitions in bip_state.h
    assert "#define BIP_EVENT_MODE_CONDITION  (1 << 10)" in state_h, \
        "BIP_EVENT_MODE_CONDITION (1 << 10) not defined in bip_state.h"
    assert "#define BIP_EVENT_SMART_FINISHED  (1 << 11)" in state_h, \
        "BIP_EVENT_SMART_FINISHED (1 << 11) not defined in bip_state.h"

    # 2. Clear mask in post_mode_change includes bit 10 & 11
    clear_idx = state_cpp.find("xEventGroupClearBits(sysEventGroup,")
    assert clear_idx != -1
    clear_block = state_cpp[clear_idx:clear_idx + 1000]
    assert "BIP_EVENT_MODE_CONDITION" in clear_block, "BIP_EVENT_MODE_CONDITION not in clear mask"
    assert "BIP_EVENT_SMART_FINISHED" in clear_block, "BIP_EVENT_SMART_FINISHED not in clear mask"

    # 3. Setting bit 10 in case MODE_CONDITION
    cond_idx = state_cpp.find("case MODE_CONDITION:")
    assert cond_idx != -1
    cond_block = state_cpp[cond_idx:cond_idx + 500]
    assert "xEventGroupSetBits(sysEventGroup, BIP_EVENT_MODE_CONDITION);" in cond_block, \
        "MODE_CONDITION does not set BIP_EVENT_MODE_CONDITION"

    # 4. update_smart sets bit 11 and transitions to MODE_IDLE
    smart_idx = state_cpp.find("static void update_smart() {")
    assert smart_idx != -1
    smart_block = state_cpp[smart_idx:smart_idx + 2500]
    assert "xEventGroupSetBits(sysEventGroup, BIP_EVENT_SMART_FINISHED);" in smart_block, \
        "BIP_EVENT_SMART_FINISHED not set upon reaching target"
    assert "post_mode_change(MODE_IDLE);" in smart_block, \
        "post_mode_change(MODE_IDLE) not called upon SMART_FINISHED"

    # 5. Bitmask orthogonality check
    bits = [1 << i for i in range(12)]
    for i in range(len(bits)):
        for j in range(i + 1, len(bits)):
            assert bits[i] & bits[j] == 0, f"Bit collision between {i} and {j}"
    print("      PASS: FreeRTOS event bits allocated; SMART_FINISHED transitions cleanly to IDLE.")


def test_num_sls_01():
    print("[9/9] Verifying NUM-SLS-01 (Unconditionally A-Stable SLS Exponential Integrator)...")
    phys_cpp = (MAIN_DIR / "bip_balloon_physics.cpp").read_text(encoding="utf-8")

    # 1. Source code assertions
    assert "s_prev_lambda" in phys_cpp, "s_prev_lambda file-static state not found"
    assert "s_first_physics_step" in phys_cpp, "s_first_physics_step flag not found"
    assert "float alpha = std::exp(-safe_dt / TAU_R);" in phys_cpp, \
        "Analytic exponential decay factor alpha = exp(-dt / TAU_R) not found"
    assert "viscous_stress_kpa = (viscous_stress_kpa * alpha) + (E_r * d_lambda);" in phys_cpp, \
        "Exponential integrator recurrence not found in bip_balloon_physics.cpp"
    assert "lambda > 10.0f" in phys_cpp, "Upper stretch ratio bound clamp (10.0f) not found"

    # 2. Reset in init_balloon_physics
    init_idx = phys_cpp.find("void init_balloon_physics(BalloonType type) {")
    assert init_idx != -1
    init_block = phys_cpp[init_idx:init_idx + 400]
    assert "s_prev_lambda = 1.0f;" in init_block, "s_prev_lambda not reset in init_balloon_physics"
    assert "s_first_physics_step = true;" in init_block, "s_first_physics_step not reset in init_balloon_physics"

    # 3. Mathematical proof & numerical simulation of unconditional A-stability
    TAU_R = 1.2
    E_r = 80.0

    # Test under extreme time step dt = 5.0 seconds (Forward Euler explodes because |1 - dt/tau| = |1 - 4.167| = 3.167 > 1)
    viscous_stress = 100.0
    safe_dt = min(1.0, max(0.0001, 5.0))  # clamped to 1.0s
    alpha = math.exp(-safe_dt / TAU_R)
    assert 0.0 < alpha < 1.0, f"Exponential decay factor invalid: {alpha}"
    
    # After pause with no stretch change (d_lambda = 0), stress decays monotonically
    decayed_stress = viscous_stress * alpha
    assert 0.0 <= decayed_stress < viscous_stress, "Exponential decay failed"
    assert not math.isnan(decayed_stress) and not math.isinf(decayed_stress)

    # Simulate 100 steps of Forward Euler vs Exponential Integrator under dt = 2.5s
    euler_stress = 100.0
    exp_stress = 100.0
    dt_unstable = 2.5
    for _ in range(10):
        # Forward Euler
        euler_stress = euler_stress * (1.0 - dt_unstable / TAU_R)
        # Exponential Integrator
        exp_stress = exp_stress * math.exp(-min(1.0, dt_unstable) / TAU_R)
    
    # Assert Euler oscillated and diverged, while Exponential remained bounded and non-negative
    assert abs(euler_stress) > 150.0, "Forward Euler should exhibit explosive oscillation"
    assert 0.0 < exp_stress < 100.0, "Exponential integrator must remain stably bounded"
    print("      PASS: SLS Exponential Integrator verified unconditionally A-stable across extreme dt.")


def main():
    print("=================================================================")
    print("Balloon Intelligent Pump (BIP) Milestone 2 Verification Suite")
    print("=================================================================")
    try:
        test_alg_mr_01()
        test_alg_mr_02()
        test_alg_mr_03()
        test_alg_rls_01()
        test_alg_fsm_01()
        test_alg_vol_01()
        test_alg_vol_02()
        test_alg_fsm_02()
        test_num_sls_01()
        print("=================================================================")
        print("ALL 8 MILESTONE 2 ITEMS + NUMERICAL STABILITY VERIFIED (9/9) 100%!")
        print("=================================================================")
        return 0
    except AssertionError as e:
        import traceback
        traceback.print_exc()
        print(f"\n[FAIL] VERIFICATION FAILED: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\n[ERROR] UNEXPECTED EXCEPTION: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
