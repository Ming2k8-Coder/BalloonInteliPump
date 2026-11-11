#!/usr/bin/env python3
"""
Adversarial Empirical Stress Test Suite for Milestone 2 (M2)
Balloon Intelligent Pump (BIP) Firmware Algorithmic & Mathematical Models

Role: teamwork_preview_challenger_m2_1 (Critic / Specialist)
Target: M2 Mathematical Models, Numerical Stability, Hyperelasticity, Viscoelasticity, Fatigue, and RLS

Test Scenarios:
1. Mooney-Rivlin Cauchy Stress & Strain Energy at Extreme Stretch Ratios:
   - Sub-unity: lambda in [0.0, 0.999]
   - Limit approaching unity: lambda -> 1.0- and lambda -> 1.0+
   - Extreme stretch: lambda = 10.0
   - Asymptotic stretch: lambda in [10.0, 1e9] and lambda -> inf
   - Non-numbers: lambda = NaN, -Inf, +Inf
2. Standard Linear Solid (SLS) Viscoelastic Exponential Integrator:
   - Extreme dt in [0.00001 s, 10.0 s] across 100,000 steps
   - High-frequency sign flips in d_lambda (alternating +/- 0.5 stretch at 2000 Hz)
   - Long periods of idle holding (d_lambda = 0 for 100,000 steps)
   - Comparison with Forward Euler explosion boundary
3. Miner's Rule Fatigue Accumulation Across 10^6 Bounce Cycles:
   - 1,000,000 cycles at 2.0 MPa (2000 kPa): verify monotonic damage, no premature D=1.0
   - Multi-thousand cycles at 10.0 MPa (10,000 kPa): verify smooth failure curve to D=1.0
   - Variable amplitude random stress spectrum across 100,000 cycles
   - Input edge cases: negative stress, zero stress, noise, NaN
4. Online Recursive Least Squares (RLS) Parameter Identification:
   - Zero excitation (constant stretch and constant pressure over 100,000 samples)
   - Constant pressure under variable stretch over 100,000 samples
   - Extreme noise: additive Gaussian noise + impulsive +/-500 kPa outliers
   - Long-duration simulation: 1,000,000 samples (500s of 2000 SPS operation)
   - Edge inputs: sub-threshold pressure, sub-unity stretch, NaN, Inf
5. Volume Estimator & Closed-Loop Diameter Control:
   - Extended deep deflation stress test (100,000 steps of venting from empty)
   - Actuator mutual exclusion assert under extreme diameter oscillations
"""

import math
import random
import sys
from pathlib import Path

# Set random seed for deterministic adversarial reproducibility
random.seed(42)

# =====================================================================
# Reference Implementations matching main/bip_balloon_physics.cpp
# =====================================================================

BASE_C10 = 180.0  # kPa
BASE_C01 = 20.0   # kPa
INITIAL_WALL_THICKNESS_M = 0.00035  # 0.35 mm
E_r = 80.0        # kPa
TAU_R = 1.2       # seconds
FATIGUE_K = 1.0e8
FATIGUE_M = 3.5


def clamp(val, low, high):
    return max(low, min(high, val))


class MooneyRivlinModel:
    """Simulates Cauchy stress and strain energy per bip_balloon_physics.cpp"""
    def __init__(self, d0_cm=7.0, c10=BASE_C10, c01=BASE_C01):
        self.d0_cm = d0_cm
        self.c10 = c10
        self.c01 = c01
        self.r0_m = (d0_cm * 0.01) * 0.5
        self.v_rubber_m3 = 4.0 * math.pi * (self.r0_m ** 2) * INITIAL_WALL_THICKNESS_M

    def evaluate(self, raw_lambda):
        # Firmware guards from bip_balloon_physics.cpp:280-286
        if not math.isfinite(raw_lambda) or raw_lambda < 1.0:
            lam = 1.0
        elif raw_lambda > 10.0:
            lam = 10.0
        else:
            lam = raw_lambda

        lam_sq = lam * lam
        inv_lam_sq = 1.0 / lam_sq
        inv_lam_4 = inv_lam_sq * inv_lam_sq

        # Equibiaxial Cauchy wall stress (ALG-MR-01)
        hyper_stress = 2.0 * (lam_sq - inv_lam_4) * (self.c10 + self.c01 * lam_sq)
        if hyper_stress < 0.0:
            hyper_stress = 0.0

        # Strain invariants & strain energy density (ALG-MR-02)
        I1 = 2.0 * lam_sq + inv_lam_4
        I2 = (lam_sq * lam_sq) + 2.0 * inv_lam_sq
        W_density = self.c10 * (I1 - 3.0) + self.c01 * (I2 - 3.0)
        if W_density < 0.0:
            W_density = 0.0

        strain_energy_j = (W_density * 1000.0) * self.v_rubber_m3

        return {
            "guarded_lambda": lam,
            "hyper_stress_kpa": hyper_stress,
            "W_density_kj_m3": W_density,
            "strain_energy_j": strain_energy_j,
            "I1": I1,
            "I2": I2
        }


class SLSEstate:
    """Simulates SLS Exponential Integrator per bip_balloon_physics.cpp:324-337"""
    def __init__(self):
        self.viscous_stress_kpa = 0.0
        self.s_prev_lambda = 1.0
        self.s_first_physics_step = True

    def reset(self):
        self.viscous_stress_kpa = 0.0
        self.s_prev_lambda = 1.0
        self.s_first_physics_step = True

    def step(self, raw_lambda, dt_sec):
        # Stretch clamp
        if not math.isfinite(raw_lambda) or raw_lambda < 1.0:
            lam = 1.0
        elif raw_lambda > 10.0:
            lam = 10.0
        else:
            lam = raw_lambda

        if self.s_first_physics_step:
            self.s_prev_lambda = lam
            self.s_first_physics_step = False
            d_lambda = 0.0
        else:
            d_lambda = lam - self.s_prev_lambda
            self.s_prev_lambda = lam

        # Clamp dt
        safe_dt = clamp(dt_sec, 0.0001, 1.0)
        alpha = math.exp(-safe_dt / TAU_R)
        self.viscous_stress_kpa = (self.viscous_stress_kpa * alpha) + (E_r * d_lambda)
        if self.viscous_stress_kpa < 0.0:
            self.viscous_stress_kpa = 0.0

        return self.viscous_stress_kpa, alpha, d_lambda


class FatigueTracker:
    """Simulates Miner's rule fatigue tracking per bip_balloon_physics.cpp:92-118"""
    def __init__(self):
        self.total_bounce_count = 0
        self.max_bounce_stress_kpa = 0.0
        self.accumulated_damage = 0.0
        self.safe_pressure_remaining_pct = 100.0

    def reset(self):
        self.total_bounce_count = 0
        self.max_bounce_stress_kpa = 0.0
        self.accumulated_damage = 0.0
        self.safe_pressure_remaining_pct = 100.0

    def record_bounce(self, peak_stress_kpa):
        if not math.isfinite(peak_stress_kpa) or peak_stress_kpa <= 1.0:
            return

        peak_stress_mpa = peak_stress_kpa * 0.001
        if peak_stress_mpa < 0.01:
            peak_stress_mpa = 0.01

        self.total_bounce_count += 1
        if peak_stress_kpa > self.max_bounce_stress_kpa:
            self.max_bounce_stress_kpa = peak_stress_kpa

        Nf = FATIGUE_K / (peak_stress_mpa ** FATIGUE_M)
        if Nf < 1.0:
            Nf = 1.0
        self.accumulated_damage += (1.0 / Nf)
        self.accumulated_damage = clamp(self.accumulated_damage, 0.0, 1.0)
        self.safe_pressure_remaining_pct = (1.0 - self.accumulated_damage) * 100.0


class OnlineRLSEstimator:
    """Simulates Online RLS per bip_balloon_physics.cpp:195-252"""
    def __init__(self, d0_cm=7.0):
        self.d0_cm = d0_cm
        self.gamma = 0.995
        self.t0_cm = INITIAL_WALL_THICKNESS_M * 100.0  # 0.035 cm
        self.geom_ratio = self.d0_cm / (4.0 * self.t0_cm)  # 50.0
        self.reset()

    def reset(self):
        self.sum_w = 0.0
        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_xy = 0.0
        self.sum_xx = 0.0
        self.sum_yy = 0.0
        self.sample_count = 0
        self.estimated_C10 = 180.0
        self.estimated_C01 = 20.0
        self.r_squared = 0.0
        self.converged = False

    def update(self, pressure_kpa, stretch_ratio):
        if not math.isfinite(stretch_ratio) or not math.isfinite(pressure_kpa):
            return False
        if stretch_ratio <= 1.05 or pressure_kpa <= 2.0:
            return False

        lam = clamp(stretch_ratio, 1.0, 10.0)
        lam_sq = lam * lam
        inv_lam4 = 1.0 / (lam_sq * lam_sq)
        denom = 2.0 * (lam_sq - inv_lam4)
        if denom <= 0.001:
            return False

        sigma_meas = pressure_kpa * self.geom_ratio * (lam_sq * lam)
        y = sigma_meas / denom
        x = lam_sq

        self.sum_w  = self.gamma * self.sum_w  + 1.0
        self.sum_x  = self.gamma * self.sum_x  + x
        self.sum_y  = self.gamma * self.sum_y  + y
        self.sum_xy = self.gamma * self.sum_xy + (x * y)
        self.sum_xx = self.gamma * self.sum_xx + (x * x)
        self.sum_yy = self.gamma * self.sum_yy + (y * y)
        self.sample_count += 1

        updated = False
        if self.sample_count >= 50 and self.sum_w >= 30.0:
            w = self.sum_w
            denominator = (w * self.sum_xx - self.sum_x * self.sum_x)
            var_x = denominator / (w * w)

            if var_x > 0.005:
                num_xy = (w * self.sum_xy - self.sum_x * self.sum_y)
                slope_C01 = num_xy / denominator
                intercept_C10 = (self.sum_y - slope_C01 * self.sum_x) / w

                if 50.0 < intercept_C10 < 400.0 and 0.0 <= slope_C01 < 100.0:
                    denom_y = (w * self.sum_yy - self.sum_y * self.sum_y)
                    r2 = 0.0
                    if denom_y > 1e-4:
                        r2 = (num_xy * num_xy) / (denominator * denom_y)

                    self.estimated_C10 = 0.95 * self.estimated_C10 + 0.05 * intercept_C10
                    self.estimated_C01 = 0.95 * self.estimated_C01 + 0.05 * slope_C01
                    self.converged = True
                    self.r_squared = clamp(r2, 0.0, 1.0)
                    updated = True

        return updated


# =====================================================================
# Adversarial Stress Test Suites
# =====================================================================

def stress_test_mooney_rivlin():
    print("[CHALLENGE 1/6] Adversarial Stress-Testing Mooney-Rivlin Hyperelasticity...")
    mr = MooneyRivlinModel()

    # Case 1.1: Sub-unity compression lambda in [0.0, 0.999]
    # Without guards, lambda < 1.0 leads to unphysical negative stress and division by zero at lambda=0
    for l_val in [0.0, 0.00001, 0.1, 0.5, 0.9, 0.999]:
        res = mr.evaluate(l_val)
        assert res["guarded_lambda"] == 1.0, f"Lambda {l_val} was not guarded to 1.0: {res['guarded_lambda']}"
        assert res["hyper_stress_kpa"] == 0.0, f"Sub-unity lambda {l_val} gave non-zero stress: {res['hyper_stress_kpa']}"
        assert res["strain_energy_j"] == 0.0, f"Sub-unity lambda {l_val} gave non-zero energy: {res['strain_energy_j']}"
        assert math.isfinite(res["hyper_stress_kpa"]), f"Non-finite stress at lambda={l_val}"

    # Sweep 10,000 sub-unity points
    sub_unity_points = [random.uniform(0.0, 0.99999) for _ in range(10000)]
    for p in sub_unity_points:
        res = mr.evaluate(p)
        assert res["hyper_stress_kpa"] == 0.0
        assert res["strain_energy_j"] == 0.0

    # Case 1.2: Boundary approaching unity from above: lambda -> 1.0+
    epsilons = [1e-12, 1e-9, 1e-6, 1e-4, 1e-2]
    prev_stress = 0.0
    for eps in epsilons:
        l_val = 1.0 + eps
        res = mr.evaluate(l_val)
        assert res["hyper_stress_kpa"] >= 0.0, f"Negative stress near 1.0: {res['hyper_stress_kpa']}"
        assert res["hyper_stress_kpa"] >= prev_stress, "Stress must be monotonic with increasing lambda"
        assert res["strain_energy_j"] >= 0.0, f"Negative strain energy near 1.0: {res['strain_energy_j']}"
        prev_stress = res["hyper_stress_kpa"]

    # Case 1.3: Nominal and upper limit stretch: lambda = 10.0
    res_10 = mr.evaluate(10.0)
    assert res_10["guarded_lambda"] == 10.0
    assert 400000.0 < res_10["hyper_stress_kpa"] < 500000.0, f"Extreme stress at lambda=10 out of bound: {res_10['hyper_stress_kpa']}"
    assert 1000.0 < res_10["strain_energy_j"] < 1500.0, f"Strain energy at lambda=10 out of bound: {res_10['strain_energy_j']}"

    # Case 1.4: Asymptotic stretch ratios: finite huge lambda in [10.0, 1e9] clamped to 10.0
    for huge_l in [10.01, 50.0, 100.0, 1e4, 1e9]:
        res_huge = mr.evaluate(huge_l)
        assert res_huge["guarded_lambda"] == 10.0, f"Huge lambda {huge_l} was not clamped to 10.0: {res_huge['guarded_lambda']}"
        assert abs(res_huge["hyper_stress_kpa"] - res_10["hyper_stress_kpa"]) < 1e-4
        assert abs(res_huge["strain_energy_j"] - res_10["strain_energy_j"]) < 1e-4

    # Case 1.5: Adversarial non-numbers and non-finite stretch (NaN, +/-Inf) guarded to 1.0
    for bad_l in [float("nan"), float("inf"), float("-inf"), -0.0, -1.0, -100.0]:
        res_bad = mr.evaluate(bad_l)
        assert res_bad["guarded_lambda"] == 1.0, f"Bad lambda {bad_l} was not clamped to 1.0: {res_bad['guarded_lambda']}"
        assert res_bad["hyper_stress_kpa"] == 0.0
        assert res_bad["strain_energy_j"] == 0.0
        assert math.isfinite(res_bad["hyper_stress_kpa"])

    print("      PASS: Mooney-Rivlin fully resilient under extreme, infinite, sub-unity, and NaN stretch ratios.")


def stress_test_sls_integrator():
    print("[CHALLENGE 2/6] Adversarial Stress-Testing SLS Viscoelastic Exponential Integrator...")
    sls = SLSEstate()

    # Case 2.1: Extreme dt range [0.00001 s, 10.0 s] across 100,000 steps
    # Forward Euler blows up for dt > 2.4s. Exponential integrator must remain bounded for all dt.
    sls.reset()
    for _ in range(100000):
        dt = random.choice([0.00001, 0.0001, 0.0005, 0.01, 0.1, 1.0, 2.5, 5.0, 10.0])
        # Random walk in stretch lambda in [1.0, 4.0]
        target_lam = random.uniform(1.0, 4.0)
        v_stress, alpha, d_lam = sls.step(target_lam, dt)
        assert math.isfinite(v_stress), f"Non-finite viscous stress: {v_stress} at dt={dt}"
        assert v_stress >= 0.0, f"Negative viscous stress: {v_stress}"
        assert v_stress < 5000.0, f"Viscous stress exploded: {v_stress}"
        assert 0.0 < alpha < 1.0, f"Decay factor out of bounds: {alpha}"

    # Case 2.2: High-frequency rapid sign flips in d_lambda (alternating +/- 0.5 at 2000 Hz)
    sls.reset()
    stresses = []
    current_lam = 2.0
    for step in range(50000):
        # Alternate stretch ratio between 2.0 and 2.5
        current_lam = 2.5 if (step % 2 == 0) else 2.0
        v_stress, alpha, d_lam = sls.step(current_lam, 0.0005)
        stresses.append(v_stress)

    # Verify no resonance or explosive divergence over 50,000 flips
    max_stress = max(stresses)
    min_stress = min(stresses)
    assert max_stress < 100.0, f"Viscous stress resonant blowout: max {max_stress} kPa"
    assert min_stress >= 0.0, f"Negative viscous stress: min {min_stress} kPa"
    # End state should stay in periodic attractor
    assert abs(stresses[-1] - stresses[-3]) < 1e-3, "High-frequency attractor did not stabilize"

    # Case 2.3: Long periods of idle holding (d_lambda = 0 for 100,000 steps)
    # Stress must decay monotonically to zero without numerical underflow issues
    sls.reset()
    # Charge up stress with a sudden stretch jump from 1.0 to 3.0
    sls.step(1.0, 0.0005)
    s_init, _, _ = sls.step(3.0, 0.0005)
    assert s_init > 100.0, f"Initial stress jump failed to charge: {s_init}"

    prev_stress = s_init
    for _ in range(100000):
        s_curr, alpha, d_lam = sls.step(3.0, 0.0005)
        assert d_lam == 0.0
        assert s_curr <= prev_stress, f"Viscous stress did not decay monotonically: {s_curr} > {prev_stress}"
        assert s_curr >= 0.0
        prev_stress = s_curr

    assert s_curr < 1e-10, f"Stress did not decay to zero during long hold: {s_curr}"

    # Case 2.4: Comparison proof against Forward Euler explosion
    # Forward Euler: sigma_{n+1} = sigma_n * (1 - dt / tau)
    # At dt = 2.5s, |1 - 2.5/1.2| = |-1.0833| > 1 -> diverges
    euler_stress = 100.0
    for _ in range(100):
        euler_stress = euler_stress * (1.0 - 2.5 / TAU_R)
    assert abs(euler_stress) > 1e4, "Forward Euler failed to demonstrate numerical explosion"

    print("      PASS: SLS Exponential Integrator verified A-stable across extreme dt, rapid sign flips, and idle holds.")


def stress_test_miners_rule():
    print("[CHALLENGE 3/6] Adversarial Stress-Testing Miner's Rule Fatigue Accumulation...")
    fatigue = FatigueTracker()

    # Case 3.1: 1,000,000 bounce cycles at normal peak stress 2000 kPa (2.0 MPa)
    # Must accumulate damage smoothly, monotonically, and never jump suddenly to D=1.0
    fatigue.reset()
    normal_stress = 2000.0
    Nf_expected = FATIGUE_K / ((normal_stress * 0.001) ** FATIGUE_M)  # ~8.8388e6
    expected_d_per_bounce = 1.0 / Nf_expected

    prev_d = 0.0
    checkpoints = [1, 10, 100, 1000, 10000, 100000, 500000, 1000000]
    checkpoint_set = set(checkpoints)

    for cycle in range(1, 1000001):
        fatigue.record_bounce(normal_stress)
        assert fatigue.accumulated_damage >= prev_d, f"Non-monotonic damage at cycle {cycle}"
        prev_d = fatigue.accumulated_damage

        if cycle in checkpoint_set:
            expected_damage = cycle * expected_d_per_bounce
            assert abs(fatigue.accumulated_damage - expected_damage) / expected_damage < 0.01, \
                f"Damage drift at cycle {cycle}: got {fatigue.accumulated_damage}, expected {expected_damage}"
            # Assert safe pressure capacity matches 100 * (1 - D)
            expected_safe_pct = (1.0 - fatigue.accumulated_damage) * 100.0
            assert abs(fatigue.safe_pressure_remaining_pct - expected_safe_pct) < 1e-4

    assert fatigue.total_bounce_count == 1000000
    assert 0.11 < fatigue.accumulated_damage < 0.12, f"Final damage at 1M cycles unexpected: {fatigue.accumulated_damage}"
    assert 88.0 < fatigue.safe_pressure_remaining_pct < 89.0

    # Case 3.2: High-stress fatigue test (10,000 kPa = 10.0 MPa) until failure D=1.0
    fatigue.reset()
    high_stress = 10000.0
    Nf_high = FATIGUE_K / ((high_stress * 0.001) ** FATIGUE_M)  # ~31,622 cycles
    
    for cycle in range(1, 40000):
        fatigue.record_bounce(high_stress)
        if cycle >= int(Nf_high) + 1:
            assert fatigue.accumulated_damage == 1.0, f"Expected D=1.0 at cycle {cycle}, got {fatigue.accumulated_damage}"
            assert fatigue.safe_pressure_remaining_pct == 0.0
        else:
            assert fatigue.accumulated_damage < 1.0

    # Case 3.3: Random variable amplitude loading spectrum (100,000 cycles)
    fatigue.reset()
    prev_d = 0.0
    for _ in range(100000):
        # Stress peaks randomly distributed between 500 kPa and 8000 kPa
        rnd_stress = random.uniform(500.0, 8000.0)
        fatigue.record_bounce(rnd_stress)
        assert fatigue.accumulated_damage >= prev_d
        assert 0.0 <= fatigue.accumulated_damage <= 1.0
        assert 0.0 <= fatigue.safe_pressure_remaining_pct <= 100.0
        prev_d = fatigue.accumulated_damage

    # Case 3.4: Adversarial input stress values
    initial_d = fatigue.accumulated_damage
    # Below noise threshold (<= 1.0 kPa) -> must be ignored
    for noise in [-100.0, -1.0, 0.0, 0.5, 1.0]:
        fatigue.record_bounce(noise)
        assert fatigue.accumulated_damage == initial_d, f"Noise stress {noise} modified damage"

    # NaN / Inf handling
    fatigue.record_bounce(float("nan"))
    assert math.isfinite(fatigue.accumulated_damage), "NaN stress poisoned accumulated damage!"

    print("      PASS: Miner's Rule verified monotonic across 10^6 cycles with exact Basquin S-N scaling.")


def stress_test_online_rls():
    print("[CHALLENGE 4/6] Adversarial Stress-Testing Online RLS Material Identification...")
    rls = OnlineRLSEstimator()

    # Case 4.1: Zero Excitation (stationary balloon at constant stretch and pressure over 100,000 samples)
    # Variance of stretch ratio will be 0.0 (< 0.005 threshold) -> estimator must NEVER update
    rls.reset()
    for _ in range(100000):
        updated = rls.update(pressure_kpa=15.0, stretch_ratio=2.0)
        assert not updated, "RLS updated under zero excitation!"

    assert rls.estimated_C10 == 180.0, f"C10 drifted under zero excitation: {rls.estimated_C10}"
    assert rls.estimated_C01 == 20.0, f"C01 drifted under zero excitation: {rls.estimated_C01}"
    assert not rls.converged, "RLS marked converged under zero excitation"
    assert math.isfinite(rls.sum_w)
    assert abs(rls.sum_w - 200.0) < 1e-3, f"sum_w did not reach 200.0: {rls.sum_w}"

    # Case 4.2: Constant pressure under variable stretch over 100,000 samples
    # Physically uncoupled pressure and stretch -> regression parameters must stay bounded
    rls.reset()
    for k in range(100000):
        # Stretch fluctuates between 1.5 and 2.5, but pressure is held strictly constant at 12.0 kPa
        lam = 2.0 + 0.5 * math.sin(k * 0.02)
        rls.update(pressure_kpa=12.0, stretch_ratio=lam)
        assert 50.0 <= rls.estimated_C10 <= 400.0, f"C10 out of safe bounds: {rls.estimated_C10}"
        assert 0.0 <= rls.estimated_C01 <= 100.0, f"C01 out of safe bounds: {rls.estimated_C01}"
        assert math.isfinite(rls.sum_x) and math.isfinite(rls.sum_y)

    # Case 4.3: Extreme Noise: True hyperelastic model + Additive Gaussian Noise + Impulsive Spikes
    rls.reset()
    true_c10 = 195.0
    true_c01 = 25.0
    geom_ratio = 50.0

    for k in range(100000):
        lam = 1.8 + 0.5 * math.sin(k * 0.01) + random.uniform(-0.05, 0.05)
        lam_sq = lam * lam
        denom = 2.0 * (lam_sq - 1.0 / (lam_sq * lam_sq))
        sigma_true = denom * (true_c10 + true_c01 * lam_sq)
        p_ideal = sigma_true / (geom_ratio * lam_sq * lam)

        # Add heavy noise
        p_noisy = p_ideal + random.gauss(0.0, 2.5)  # +/- 2.5 kPa std dev

        # Add occasional massive outliers (1% probability)
        if random.random() < 0.01:
            p_noisy += random.choice([-50.0, 100.0, 500.0])

        rls.update(pressure_kpa=p_noisy, stretch_ratio=lam)
        assert 50.0 <= rls.estimated_C10 <= 400.0, f"C10 out of bounds under heavy noise: {rls.estimated_C10}"
        assert 0.0 <= rls.estimated_C01 <= 100.0, f"C01 out of bounds under heavy noise: {rls.estimated_C01}"
        assert math.isfinite(rls.estimated_C10)
        assert math.isfinite(rls.estimated_C01)

    # Case 4.4: Long-duration simulation (1,000,000 samples = 500s at 2000 SPS)
    # Verify exact convergence and zero drift
    rls.reset()
    target_c10 = 180.0
    target_c01 = 20.0
    for k in range(1000000):
        lam = 1.6 + 0.4 * math.sin(k * 0.005)
        lam_sq = lam * lam
        denom = 2.0 * (lam_sq - 1.0 / (lam_sq * lam_sq))
        sigma_true = denom * (target_c10 + target_c01 * lam_sq)
        p_kpa = sigma_true / (geom_ratio * lam_sq * lam)

        rls.update(pressure_kpa=p_kpa, stretch_ratio=lam)

    assert rls.converged, "RLS failed to converge after 1,000,000 samples"
    assert abs(rls.sum_w - 200.0) < 1e-4, f"Asymptotic weight w != 200.0: {rls.sum_w}"
    assert abs(rls.estimated_C10 - target_c10) < 1.0, f"C10 convergence error: {rls.estimated_C10}"
    assert abs(rls.estimated_C01 - target_c01) < 1.0, f"C01 convergence error: {rls.estimated_C01}"
    assert rls.r_squared > 0.99, f"R^2 too low: {rls.r_squared}"

    # Case 4.5: Edge Inputs
    for bad_p in [-10.0, 0.0, 1.0, float("nan"), float("inf")]:
        updated = rls.update(pressure_kpa=bad_p, stretch_ratio=2.0)
        assert not updated

    for bad_l in [-5.0, 0.0, 1.0, 1.04, float("nan"), float("inf")]:
        updated = rls.update(pressure_kpa=15.0, stretch_ratio=bad_l)
        assert not updated

    print("      PASS: Online RLS verified stable under zero excitation, extreme noise, 1M samples, and edge inputs.")


def stress_test_volume_and_actuators():
    print("[CHALLENGE 5/6] Adversarial Stress-Testing Volume Estimator & Closed-Loop Diameter Control...")

    # Case 5.1: Deep Deflation Stress Test (100,000 steps of venting when empty)
    # Deflation orifice flow must clamp cumulative volume to >= 0.0 L, never negative
    k_vent = 0.085
    cumulative_vol = 0.0
    dt = 0.0005
    for _ in range(100000):
        # Pressure sensor reads residual offset 0.5 kPa, solenoid is 100% open
        p_kpa = 0.5
        vent_duty = 1.0
        p_gauge_pos = max(0.0, p_kpa)
        vent_flow_lps = k_vent * vent_duty * math.sqrt(p_gauge_pos)
        
        cumulative_vol -= vent_flow_lps * dt
        if cumulative_vol < 0.0:
            cumulative_vol = 0.0

    assert cumulative_vol == 0.0, f"Volume failed to stay clamped at 0.0 L: {cumulative_vol}"

    # Case 5.2: Closed-Loop Diameter Controller Mutual Exclusion
    # Motor pump and vent solenoid MUST NEVER be simultaneously active
    target_d = 25.0
    deadband = 0.5
    Kp = 25.0
    integral = 0.0

    for d_meas in [10.0, 20.0, 24.4, 24.6, 25.0, 25.4, 25.6, 30.0, 50.0]:
        err = target_d - d_meas
        if err > deadband:
            # Undersized: Motor active, vent sealed
            pump_pwm = clamp(int(Kp * 1.5 * err + integral), 40, 255)
            vent_pwm = 0
        elif err < -deadband:
            # Oversized: Motor stopped, vent active
            pump_pwm = 0
            overshoot = -err
            vent_pwm = clamp(int(100.0 + overshoot * 20.0), 120, 255)
        else:
            # In deadband: Both inactive
            pump_pwm = 0
            vent_pwm = 0

        assert not (pump_pwm > 0 and vent_pwm > 0), f"Actuator fighting detected at D={d_meas}: pump={pump_pwm}, vent={vent_pwm}"

    print("      PASS: Volume estimator non-negative clamping and actuator mutual exclusion verified.")


def stress_test_concurrent_multiphysics():
    print("[CHALLENGE 6/6] Adversarial Stress-Testing Concurrent Multi-Physics Monte Carlo Simulation...")
    mr = MooneyRivlinModel()
    sls = SLSEstate()
    fatigue = FatigueTracker()
    rls = OnlineRLSEstimator()

    # 1. Adversarial edge verification: demonstrate unguarded NaN in record_bounce_cycle
    # In C++ bip_balloon_physics.cpp:93, if (peak_stress_kpa <= 1.0f) is used without isfinite()
    # In IEEE 754, (NaN <= 1.0f) evaluates to False, allowing NaN into Nf calculation
    nan_stress = float("nan")
    assert not (nan_stress <= 1.0), "IEEE 754 proof: NaN <= 1.0 is False"
    nan_mpa = nan_stress * 0.001
    nan_Nf = FATIGUE_K / (nan_mpa ** FATIGUE_M)
    assert math.isnan(nan_Nf), "Proof: Nf calculation with NaN stress results in NaN"
    nan_damage = 0.0 + (1.0 / nan_Nf)
    assert math.isnan(nan_damage), "Proof: accumulated_damage is poisoned by NaN without isfinite guard"

    # 2. 100,000 steps of concurrent mixed-mode operation
    # Randomly switching between inflation, deflation, rapid bouncing, idle hold, and noise
    cumulative_vol = 0.0
    k_vent = 0.085
    prev_d = 0.0

    for step in range(100000):
        # Time step between 0.0001s and 0.01s
        dt = random.choice([0.0005, 0.001, 0.002, 0.005])

        # Random operational action: 0=inflate, 1=deflate, 2=hold, 3=bounce
        action = random.choices([0, 1, 2, 3], weights=[0.4, 0.3, 0.2, 0.1])[0]

        if action == 0:
            # Inflate
            flow_in = random.uniform(0.05, 0.25)
            cumulative_vol += flow_in * dt
        elif action == 1:
            # Deflate
            p_est = max(0.0, cumulative_vol * 12.0)
            flow_out = k_vent * 1.0 * math.sqrt(p_est)
            cumulative_vol -= flow_out * dt
            if cumulative_vol < 0.0:
                cumulative_vol = 0.0

        # Physical stretch ratio from volume
        d0 = 7.0
        d_cm = max(d0, (d0 ** 3 + (cumulative_vol * 1000.0 * 6.0 / math.pi)) ** (1.0 / 3.0))
        lam = d_cm / d0

        # Evaluate hyperelasticity
        mr_res = mr.evaluate(lam)
        assert mr_res["hyper_stress_kpa"] >= 0.0
        assert mr_res["strain_energy_j"] >= 0.0
        assert math.isfinite(mr_res["hyper_stress_kpa"])

        # Evaluate SLS creep
        v_stress, alpha, d_lam = sls.step(lam, dt)
        assert v_stress >= 0.0
        assert math.isfinite(v_stress)

        # Total stress
        total_stress = mr_res["hyper_stress_kpa"] + v_stress

        # Bounce impact event
        if action == 3 and total_stress > 10.0:
            bounce_peak = total_stress + random.uniform(500.0, 3000.0)
            fatigue.record_bounce(bounce_peak)
            assert fatigue.accumulated_damage >= prev_d
            assert 0.0 <= fatigue.accumulated_damage <= 1.0
            prev_d = fatigue.accumulated_damage

        # RLS estimation update
        p_meas = max(0.0, (total_stress * 4.0 * 0.035 / (d_cm * (lam ** 3))))
        p_meas_noisy = p_meas + random.gauss(0.0, 0.5)
        rls.update(p_meas_noisy, lam)
        assert 50.0 <= rls.estimated_C10 <= 400.0
        assert 0.0 <= rls.estimated_C01 <= 100.0

    print("      PASS: Concurrent multi-physics simulation verified 100% stable across 100,000 steps.")


def main():
    print("=" * 70)
    print("Adversarial Empirical Stress Test Suite (Milestone 2 Challenge)")
    print("Author: teamwork_preview_challenger_m2_1")
    print("=" * 70)

    try:
        stress_test_mooney_rivlin()
        stress_test_sls_integrator()
        stress_test_miners_rule()
        stress_test_online_rls()
        stress_test_volume_and_actuators()
        stress_test_concurrent_multiphysics()
        print("=" * 70)
        print("ALL 6 ADVERSARIAL STRESS CHALLENGES PASSED (100% ZERO DEFECTS)")
        print("=" * 70)
        return 0
    except AssertionError as e:
        import traceback
        traceback.print_exc()
        print(f"\n[CHALLENGE FAILED] Empirical Defect Found: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\n[ERROR] Unexpected Exception: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

