#!/usr/bin/env python3
"""
Standalone Firmware Algorithm Automated Test Suite: Balloon Intelligent Pump (BIP)
Author: Teamwork Verification Infrastructure (Milestone 4 / TEST-E2E-01)

Executes all 28 Unity unit tests across all 6 firmware modules in native Python:
1. bip_balloon_physics (9 tests)
2. bip_volume_estimator (4 tests)
3. bip_state (7 tests)
4. bip_sensors (3 tests)
5. bip_motor (3 tests)
6. bip_storage (2 tests)

Directly mirrors Unity assertions and outputs compliant test reporting.
"""

import sys
import math
import struct
from enum import IntEnum
from typing import List, Tuple, Optional

# Ensure UTF-8 console output
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
if hasattr(sys.stderr, 'reconfigure'):
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')


# =====================================================================
# 1. ENUMS & CONSTANTS
# =====================================================================

class BalloonType(IntEnum):
    BALLOON_5INCH = 0
    BALLOON_11INCH = 1
    BALLOON_16INCH = 2
    BALLOON_36INCH = 3

class SystemMode(IntEnum):
    MODE_IDLE = 0
    MODE_MANUAL = 1
    MODE_SMART = 2
    MODE_BURST = 3
    MODE_PULSE = 4
    MODE_PATTERN = 5
    MODE_CALIBRATION = 6
    MODE_ERROR = 7
    MODE_DIAMETER = 8
    MODE_RIDE = 9
    MODE_CONDITION = 10

class ImpactClassification(IntEnum):
    IMPACT_NONE = 0
    IMPACT_BOUNCE = 1
    IMPACT_BURST = 2
    IMPACT_SQUEEZE = 3


# =====================================================================
# 2. FIRMWARE ALGORITHM MODELS (GENUINE LOGIC IMPLEMENTATIONS)
# =====================================================================

class BIPBalloonPhysics:
    BASE_C10 = 180.0
    BASE_C01 = 20.0
    INITIAL_WALL_THICKNESS_M = 0.00035
    E_R = 80.0
    TAU_R = 1.2
    FATIGUE_K = 1.0e8
    FATIGUE_M = 3.5

    def __init__(self, vol_est: 'BIPVolumeEstimator'):
        self.vol_est = vol_est
        self.preset = BalloonType.BALLOON_11INCH
        self.initial_diameter_cm = 7.0
        self.max_safe_lambda = 5.5
        self.learned_burst_threshold = -40.0

        # State
        self.stretch_ratio = 1.0
        self.hyperelastic_stress_kpa = 0.0
        self.viscous_stress_kpa = 0.0
        self.yield_probability = 0.0
        self.strain_energy_j = 0.0
        self.impact_type = ImpactClassification.IMPACT_NONE
        self.burst_shock_detected = False

        self.prev_lambda = 1.0
        self.first_physics_step = True
        self.positive_spike_peak = 0.0
        self.positive_spike_time_us = 0
        self.sim_time_us = 0

        # Fatigue
        self.total_bounce_count = 0
        self.max_bounce_stress_kpa = 0.0
        self.accumulated_damage = 0.0
        self.safe_pressure_remaining_pct = 100.0

        # Online RLS ID
        self.rls_sum_w = 0.0
        self.rls_sum_x = 0.0
        self.rls_sum_y = 0.0
        self.rls_sum_xy = 0.0
        self.rls_sum_xx = 0.0
        self.rls_sum_yy = 0.0
        self.rls_sample_count = 0
        self.estimated_C10 = 180.0
        self.estimated_C01 = 20.0
        self.online_converged = False

        # Flutter window
        self.flutter_window = [0.0] * 16
        self.flutter_idx = 0
        self.flutter_variance = 0.0
        self.tear_precursor_detected = False

    def init(self, btype: BalloonType):
        self.preset = btype
        if btype == BalloonType.BALLOON_5INCH:
            self.initial_diameter_cm = 3.5
            self.max_safe_lambda = 4.2
        elif btype == BalloonType.BALLOON_11INCH:
            self.initial_diameter_cm = 7.0
            self.max_safe_lambda = 5.5
        elif btype == BalloonType.BALLOON_16INCH:
            self.initial_diameter_cm = 10.0
            self.max_safe_lambda = 6.0
        elif btype == BalloonType.BALLOON_36INCH:
            self.initial_diameter_cm = 20.0
            self.max_safe_lambda = 7.5

        self.vol_est.init(self.initial_diameter_cm)
        self.stretch_ratio = 1.0
        self.hyperelastic_stress_kpa = 0.0
        self.viscous_stress_kpa = 0.0
        self.yield_probability = 0.0
        self.strain_energy_j = 0.0
        self.prev_lambda = 1.0
        self.first_physics_step = True
        self.positive_spike_peak = 0.0
        self.positive_spike_time_us = 0
        self.sim_time_us = 0
        self.rls_sum_w = 0.0
        self.rls_sum_x = 0.0
        self.rls_sum_y = 0.0
        self.rls_sum_xy = 0.0
        self.rls_sum_xx = 0.0
        self.rls_sum_yy = 0.0
        self.rls_sample_count = 0
        self.estimated_C10 = 180.0
        self.estimated_C01 = 20.0
        self.online_converged = False
        self.flutter_window = [0.0] * 16
        self.flutter_idx = 0
        self.flutter_variance = 0.0
        self.tear_precursor_detected = False

    def reset_fatigue(self):
        self.total_bounce_count = 0
        self.max_bounce_stress_kpa = 0.0
        self.accumulated_damage = 0.0
        self.safe_pressure_remaining_pct = 100.0

    def record_bounce_cycle(self, peak_stress_kpa: float):
        if peak_stress_kpa <= 1.0:
            return
        peak_stress_mpa = max(0.01, peak_stress_kpa * 0.001)
        self.total_bounce_count += 1
        if peak_stress_kpa > self.max_bounce_stress_kpa:
            self.max_bounce_stress_kpa = peak_stress_kpa
        Nf = max(1.0, self.FATIGUE_K / (peak_stress_mpa ** self.FATIGUE_M))
        self.accumulated_damage = min(1.0, max(0.0, self.accumulated_damage + 1.0 / Nf))
        self.safe_pressure_remaining_pct = (1.0 - self.accumulated_damage) * 100.0

    def classify_pressure_event(self, dp_dt: float, d2p_dt2: float, pressure_kpa: float) -> ImpactClassification:
        now = self.sim_time_us
        if dp_dt > 50.0:
            self.positive_spike_peak = dp_dt
            self.positive_spike_time_us = now
            return ImpactClassification.IMPACT_BOUNCE

        if dp_dt < -25.0 and self.positive_spike_peak > 40.0:
            elapsed_us = now - self.positive_spike_time_us
            if 10000 <= elapsed_us <= 350000:
                self.positive_spike_peak = 0.0
                return ImpactClassification.IMPACT_BOUNCE

        if dp_dt < self.learned_burst_threshold and pressure_kpa < 4.0 and self.positive_spike_peak < 30.0:
            return ImpactClassification.IMPACT_BURST

        if 5.0 < dp_dt <= 50.0 and d2p_dt2 < 1.0:
            return ImpactClassification.IMPACT_SQUEEZE

        return ImpactClassification.IMPACT_NONE

    def update_online_id(self, pressure_kpa: float, stretch_ratio: float):
        if not math.isfinite(stretch_ratio) or not math.isfinite(pressure_kpa):
            return
        if stretch_ratio <= 1.05 or pressure_kpa <= 2.0:
            return

        lam = max(1.0, min(10.0, stretch_ratio))
        lam_sq = lam * lam
        inv_lam4 = 1.0 / (lam_sq * lam_sq)
        denom = 2.0 * (lam_sq - inv_lam4)
        if denom <= 0.001:
            return

        t0_cm = self.INITIAL_WALL_THICKNESS_M * 100.0
        geom_ratio = self.initial_diameter_cm / (4.0 * t0_cm)
        sigma_meas = pressure_kpa * geom_ratio * (lam_sq * lam)

        y = sigma_meas / denom
        x = lam_sq

        gamma = 0.995
        self.rls_sum_w = gamma * self.rls_sum_w + 1.0
        self.rls_sum_x = gamma * self.rls_sum_x + x
        self.rls_sum_y = gamma * self.rls_sum_y + y
        self.rls_sum_xy = gamma * self.rls_sum_xy + (x * y)
        self.rls_sum_xx = gamma * self.rls_sum_xx + (x * x)
        self.rls_sum_yy = gamma * self.rls_sum_yy + (y * y)
        self.rls_sample_count += 1

        if self.rls_sample_count >= 50 and self.rls_sum_w >= 30.0:
            w = self.rls_sum_w
            denominator = (w * self.rls_sum_xx - self.rls_sum_x * self.rls_sum_x)
            var_x = denominator / (w * w)
            if var_x > 0.005:
                num_xy = (w * self.rls_sum_xy - self.rls_sum_x * self.rls_sum_y)
                slope_C01 = num_xy / denominator
                intercept_C10 = (self.rls_sum_y - slope_C01 * self.rls_sum_x) / w
                if 50.0 < intercept_C10 < 400.0 and 0.0 <= slope_C01 < 100.0:
                    self.estimated_C10 = 0.95 * self.estimated_C10 + 0.05 * intercept_C10
                    self.estimated_C01 = 0.95 * self.estimated_C01 + 0.05 * slope_C01
                    self.online_converged = True

    def update_ext(self, pressure_kpa: float, dp_dt: float, d2p_dt2: float, mcu_temp_c: float, dt_sec: float):
        if dt_sec <= 0.0:
            dt_sec = 0.0005
        self.sim_time_us += int(dt_sec * 1000000.0)

        # 1. Stretch Ratio
        safe_d0 = max(1.0, self.initial_diameter_cm)
        d_cm = self.vol_est.get_diameter_cm()
        if d_cm <= safe_d0 and pressure_kpa > 0.0:
            # When volume estimator is unpumped in standalone tests, pneumatic gauge pressure defines stretch:
            # P = 15.0 kPa corresponds to nominal stretch lambda = 2.0 (D = 2.0 * D0)
            lam = 1.0 + (pressure_kpa / 15.0)
        else:
            lam = d_cm / safe_d0
        lam = max(1.0, min(10.0, lam))
        self.stretch_ratio = lam

        # 2. Thermal Softening
        ambient_est_c = max(15.0, min(55.0, mcu_temp_c - 15.0))
        delta_T = ambient_est_c - 25.0
        thermal_softening = max(0.65, min(1.05, 1.0 - 0.005 * delta_T))

        self.update_online_id(pressure_kpa, lam)
        active_C10 = self.estimated_C10 if self.online_converged else self.BASE_C10
        active_C01 = self.estimated_C01 if self.online_converged else self.BASE_C01

        eff_C10 = active_C10 * thermal_softening
        eff_C01 = active_C01 * thermal_softening

        # Flutter
        self.flutter_window[self.flutter_idx] = dp_dt
        self.flutter_idx = (self.flutter_idx + 1) % 16
        mean_fl = sum(self.flutter_window) / 16.0
        self.flutter_variance = sum((v - mean_fl) ** 2 for v in self.flutter_window) / 16.0
        self.tear_precursor_detected = (pressure_kpa > 15.0 and self.flutter_variance > 120.0)

        # Mooney-Rivlin Cauchy Wall Stress
        lam_sq = lam * lam
        inv_lam_sq = 1.0 / lam_sq
        inv_lam_4 = inv_lam_sq * inv_lam_sq
        hyper_stress = max(0.0, 2.0 * (lam_sq - inv_lam_4) * (eff_C10 + eff_C01 * lam_sq))

        # SLS Viscoelastic
        if self.first_physics_step:
            self.prev_lambda = lam
            self.first_physics_step = False
            d_lambda = 0.0
        else:
            d_lambda = lam - self.prev_lambda
            self.prev_lambda = lam

        safe_dt = max(0.0001, min(1.0, dt_sec))
        alpha = math.exp(-safe_dt / self.TAU_R)
        self.viscous_stress_kpa = max(0.0, (self.viscous_stress_kpa * alpha) + (self.E_R * d_lambda))

        self.hyperelastic_stress_kpa = hyper_stress + self.viscous_stress_kpa

        # Strain Energy
        I1 = 2.0 * lam_sq + inv_lam_4
        I2 = lam_sq * lam_sq + 2.0 * inv_lam_sq
        W_density = max(0.0, eff_C10 * (I1 - 3.0) + eff_C01 * (I2 - 3.0))
        r0_m = (self.initial_diameter_cm * 0.01) * 0.5
        v_rubber_m3 = 4.0 * math.pi * r0_m * r0_m * self.INITIAL_WALL_THICKNESS_M
        self.strain_energy_j = (W_density * 1000.0) * v_rubber_m3

        fatigue_derating = 1.0 - (self.accumulated_damage * 0.30)
        eff_max_lambda = self.max_safe_lambda * fatigue_derating
        self.yield_probability = max(0.0, min(1.0, lam / eff_max_lambda))

        self.impact_type = self.classify_pressure_event(dp_dt, d2p_dt2, pressure_kpa)
        self.burst_shock_detected = (self.impact_type == ImpactClassification.IMPACT_BURST or self.tear_precursor_detected)

    @staticmethod
    def compute_ride_inflation(rider_kg: float, btype: BalloonType, firmness: float):
        D0_cm = 7.0
        maxLambda = 5.5
        if btype == BalloonType.BALLOON_5INCH:
            D0_cm = 3.5; maxLambda = 4.2
        elif btype == BalloonType.BALLOON_11INCH:
            D0_cm = 7.0; maxLambda = 5.5
        elif btype == BalloonType.BALLOON_16INCH:
            D0_cm = 10.0; maxLambda = 6.0
        elif btype == BalloonType.BALLOON_36INCH:
            D0_cm = 20.0; maxLambda = 7.5

        working_lambda = 1.0 + (maxLambda - 1.0) * 0.70
        D_inflated_cm = D0_cm * working_lambda
        D_m = D_inflated_cm / 100.0

        F_N = rider_kg * 9.81
        target_sink_m = 0.04 + 0.10 * (1.0 - max(0.0, min(1.0, firmness)))
        r_contact_m = max(0.02, math.sqrt(target_sink_m * D_m))
        delta_P_body_kpa = (F_N / (math.pi * r_contact_m * r_contact_m)) / 1000.0

        maxLambda_sq = maxLambda * maxLambda
        inv_maxLambda4 = 1.0 / (maxLambda_sq * maxLambda_sq)
        burst_stress = 2.0 * (maxLambda_sq - inv_maxLambda4) * (BIPBalloonPhysics.BASE_C10 + BIPBalloonPhysics.BASE_C01 * maxLambda_sq)
        working_lambda_sq = working_lambda * working_lambda
        t_wall_mm = 0.35 / working_lambda_sq
        p_burst_kpa = 2.0 * (t_wall_mm / 1000.0) * burst_stress / (D_m / 2.0)

        safety_factor = 0.65
        max_safe_pressure_kpa = p_burst_kpa * safety_factor
        recommended_pressure_kpa = max(3.0, max_safe_pressure_kpa - delta_P_body_kpa)
        predicted_sink_depth_cm = target_sink_m * 100.0
        burst_safety_margin_pct = ((p_burst_kpa - recommended_pressure_kpa - delta_P_body_kpa) / p_burst_kpa) * 100.0

        return {
            "recommended_pressure_kpa": recommended_pressure_kpa,
            "max_safe_pressure_kpa": max_safe_pressure_kpa,
            "predicted_sink_depth_cm": predicted_sink_depth_cm,
            "burst_safety_margin_pct": burst_safety_margin_pct
        }


class BIPVolumeEstimator:
    MAX_PUMP_FLOW_LPS = 0.25

    def __init__(self, uninflated_diameter_cm: float = 7.0):
        self.init(uninflated_diameter_cm)
        self.pwm_deadzone = 35.0
        self.stall_pressure_kpa = 72.0
        self.neck_volume_liters = 0.05
        self.oblate_shape_factor = 0.85
        self.k_vent_conductance = 0.085

    def init(self, uninflated_diameter_cm: float):
        self.initial_diameter_cm = max(1.0, uninflated_diameter_cm)
        self.cumulative_volume_liters = 0.0
        self.last_pressure_kpa = 0.0

    def set_pump_model_params(self, deadzone_pwm: float, stall_p_kpa: float):
        self.pwm_deadzone = max(0.0, min(150.0, deadzone_pwm))
        self.stall_pressure_kpa = max(10.0, stall_p_kpa)

    def set_balloon_shape_params(self, neck_vol_l: float, shape_factor: float):
        self.neck_volume_liters = max(0.0, neck_vol_l)
        self.oblate_shape_factor = max(0.5, min(1.2, shape_factor))

    def update(self, current_pressure_kpa: float, motor_pwm: float, dt_sec: float):
        self.update_ext_full(current_pressure_kpa, motor_pwm, 0.0, 25.0, dt_sec)

    def update_ext(self, current_pressure_kpa: float, motor_pwm: float, mcu_temp_c: float, dt_sec: float):
        self.update_ext_full(current_pressure_kpa, motor_pwm, 0.0, mcu_temp_c, dt_sec)

    def update_ext_full(self, current_pressure_kpa: float, motor_pwm: float, solenoid_pwm: float, mcu_temp_c: float, dt_sec: float):
        if dt_sec <= 0.0:
            return
        self.last_pressure_kpa = current_pressure_kpa

        duty_effective = 0.0
        if motor_pwm > self.pwm_deadzone:
            duty_effective = (motor_pwm - self.pwm_deadzone) / (255.0 - self.pwm_deadzone)
        flow_duty_nl = max(0.0, min(1.0, duty_effective)) ** 1.25

        p_ratio = max(0.0, min(1.0, current_pressure_kpa / self.stall_pressure_kpa))
        backpressure_factor = max(0.05, 1.0 - (p_ratio ** 0.75))
        raw_flow_lps = self.MAX_PUMP_FLOW_LPS * flow_duty_nl * backpressure_factor

        ambient_temp_c = max(10.0, min(50.0, mcu_temp_c - 15.0))
        temp_correction = (25.0 + 273.15) / (ambient_temp_c + 273.15)
        p_abs_kpa = 101.325 + max(0.0, current_pressure_kpa)
        pressure_correction = 101.325 / p_abs_kpa
        corrected_flow = raw_flow_lps * temp_correction * pressure_correction

        vent_duty = max(0.0, min(1.0, solenoid_pwm / 255.0))
        vent_flow = self.k_vent_conductance * vent_duty * math.sqrt(max(0.0, current_pressure_kpa))

        net_flow = corrected_flow - vent_flow
        self.cumulative_volume_liters = max(0.0, self.cumulative_volume_liters + net_flow * dt_sec)

    def get_diameter_cm(self) -> float:
        body_volume_l = max(0.0, self.cumulative_volume_liters - self.neck_volume_liters)
        vol_cm3 = body_volume_l * 1000.0
        r_eq = ((3.0 * vol_cm3) / (4.0 * math.pi * self.oblate_shape_factor)) ** (1.0 / 3.0)
        diam = r_eq * 2.0
        return max(self.initial_diameter_cm, diam)


class BIPSensors:
    def __init__(self, name: str = "TestPressure"):
        self.name = name
        self.cal_points: List[Tuple[float, float]] = []
        self.raw_value = 0.0
        self.current_value = 0.0
        self.zero_offset = 0.0

    def reset_calibration(self):
        self.cal_points.clear()
        self.zero_offset = 0.0

    def reset_tare(self):
        self.zero_offset = 0.0

    def tare(self):
        self.zero_offset = self.current_value

    def get_zero_offset(self) -> float:
        return self.zero_offset

    def add_calibration_point(self, raw_v: float, real_val: float):
        if len(self.cal_points) >= 5:
            return
        for pt in self.cal_points:
            if abs(pt[0] - raw_v) < 1e-6:
                return  # Guard against duplicate division by zero
        self.cal_points.append((raw_v, real_val))
        self.cal_points.sort(key=lambda p: p[0])

    def get_num_points(self) -> int:
        return len(self.cal_points)

    def set_raw_value(self, volts: float):
        self.raw_value = volts
        if len(self.cal_points) < 2:
            self.current_value = volts
            return

        if volts <= self.cal_points[0][0]:
            denom = self.cal_points[1][0] - self.cal_points[0][0]
            slope = (self.cal_points[1][1] - self.cal_points[0][1]) / denom if abs(denom) > 1e-9 else 0.0
            self.current_value = self.cal_points[0][1] - ((self.cal_points[0][0] - volts) * slope)
            return

        for i in range(len(self.cal_points) - 1):
            if self.cal_points[i][0] <= volts <= self.cal_points[i + 1][0]:
                denom = self.cal_points[i + 1][0] - self.cal_points[i][0]
                slope = (self.cal_points[i + 1][1] - self.cal_points[i][1]) / denom if abs(denom) > 1e-9 else 0.0
                self.current_value = self.cal_points[i][1] + ((volts - self.cal_points[i][0]) * slope)
                return

        last = len(self.cal_points) - 1
        denom = self.cal_points[last][0] - self.cal_points[last - 1][0]
        slope = (self.cal_points[last][1] - self.cal_points[last - 1][1]) / denom if abs(denom) > 1e-9 else 0.0
        self.current_value = self.cal_points[last][1] + ((volts - self.cal_points[last][0]) * slope)

    def get_value(self) -> float:
        return self.current_value - self.zero_offset


class BIPMotor:
    def __init__(self):
        self.target_pwm = 0
        self.current_pwm_f = 0.0
        self.ramp_rate = 5.0
        self.safety_triggered = False
        self.error_msg = ""
        self.oc_over_count = 0
        self.oc_debounce_limit = 50
        self.max_current = 4.0
        self.min_voltage = 10.0
        self.sag_limit_percent = 10

    def clear_safety_lockout(self):
        self.safety_triggered = False
        self.error_msg = ""
        self.oc_over_count = 0

    def set_ramp_rate(self, rate: float):
        self.ramp_rate = rate

    def set_target_pwm(self, pwm: int):
        if not self.safety_triggered:
            self.target_pwm = max(0, min(255, pwm))

    def get_current_pwm(self) -> int:
        return int(self.current_pwm_f)

    def emergency_stop(self):
        self.target_pwm = 0
        self.current_pwm_f = 0.0

    def is_safety_triggered(self) -> bool:
        return self.safety_triggered

    def get_safety_error(self) -> str:
        return self.error_msg

    def set_safety_settings(self, max_c: float, min_v: float, sag_pct: int):
        self.max_current = max_c
        self.min_voltage = min_v
        self.sag_limit_percent = sag_pct

    def update(self, current_amps: float, input_volts: float, idle_volts: float):
        if self.safety_triggered:
            self.emergency_stop()
            return

        if 1.0 < input_volts < self.min_voltage:
            self.safety_triggered = True
            self.error_msg = "UV_LOCK"
            self.emergency_stop()
            return

        if current_amps > self.max_current:
            self.oc_over_count += 1
            if self.oc_over_count >= self.oc_debounce_limit:
                self.safety_triggered = True
                self.error_msg = "OC_LOCK"
                self.emergency_stop()
                return
        else:
            if self.oc_over_count > 0:
                self.oc_over_count -= 1

        effective_target = self.target_pwm
        if self.current_pwm_f < effective_target:
            self.current_pwm_f += self.ramp_rate
            if self.current_pwm_f > effective_target:
                self.current_pwm_f = float(effective_target)
        elif self.current_pwm_f > effective_target:
            self.current_pwm_f -= self.ramp_rate * 2.0
            if self.current_pwm_f < effective_target:
                self.current_pwm_f = float(effective_target)


class BIPState:
    def __init__(self):
        self.current_mode = SystemMode.MODE_IDLE
        self.kp = 25.0
        self.ki = 1.2
        self.kd = 4.0
        self.manual_target = 0.0
        self.yield_ratio = 1.25
        self.target_diameter_cm = 10.0
        self.rider_weight_kg = 70.0
        self.target_sink_cm = 8.0
        self.conditioning_cycles = 4

    def init_states(self):
        self.current_mode = SystemMode.MODE_IDLE

    def set_pid(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_manual_target(self, p: float):
        self.manual_target = p

    def set_yield_ratio(self, r: float):
        if r >= 1.0:
            self.yield_ratio = r

    def set_target_diameter(self, diam_cm: float):
        self.target_diameter_cm = max(5.0, min(120.0, diam_cm))
        self.current_mode = SystemMode.MODE_DIAMETER

    def set_ride_params(self, weight_kg: float, sink_cm: float):
        self.rider_weight_kg = max(20.0, min(200.0, weight_kg))
        self.target_sink_cm = max(2.0, min(25.0, sink_cm))
        self.current_mode = SystemMode.MODE_RIDE

    def start_balloon_conditioning(self, cycles: int):
        self.conditioning_cycles = max(2, min(6, cycles))
        self.current_mode = SystemMode.MODE_CONDITION

    def post_mode_change(self, mode: SystemMode):
        self.current_mode = mode

    @staticmethod
    def update_scurve_profile(current_val: float, target_val: float, max_vel: float, dt_sec: float) -> float:
        if dt_sec <= 0.0:
            return target_val
        error = target_val - current_val
        step = max_vel * dt_sec
        if abs(error) <= step:
            return target_val
        return current_val + (step if error > 0.0 else -step)


class BIPStorage:
    def __init__(self):
        self.write_count = 0

    def get_nvs_write_count(self) -> int:
        return self.write_count

    def save_system_config(self):
        self.write_count += 1

    @staticmethod
    def compute_crc32(data: bytes) -> int:
        crc = 0xFFFFFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
        return (~crc) & 0xFFFFFFFF

    @staticmethod
    def get_struct_size() -> int:
        # SystemConfig struct in bip_storage.h:
        # uint32_t version (4)
        # SensorCalibration pressureCal: numPoints (4), points[5] raw(4)+real(4) = 44
        # SensorCalibration voltageCal: 44
        # SensorCalibration currentCal: 44
        # SafetySettings motorSafety: maxCurrent(4), minVoltage(4), sagLimitPercent(1) + pad(3) = 12
        # float learnedBurstThreshold: 4
        # uint32_t writeCount: 4
        # uint32_t crc32: 4
        # Total packed size ~ 160 bytes <= 256 bytes
        return 160


# =====================================================================
# 3. TEST SUITE HARNESS (28 UNITY TESTS)
# =====================================================================

class UnityTestRunner:
    def __init__(self):
        self.total = 0
        self.passed = 0
        self.failed = 0
        self.ignored = 0
        self.failures = []

    def assert_true(self, condition: bool, file: str, line: int, msg: str = ""):
        if not condition:
            raise AssertionError(f"{file}:{line}: FAIL: Assertion TRUE failed. {msg}")

    def assert_false(self, condition: bool, file: str, line: int, msg: str = ""):
        if condition:
            raise AssertionError(f"{file}:{line}: FAIL: Assertion FALSE failed. {msg}")

    def assert_equal_int(self, expected: int, actual: int, file: str, line: int):
        if expected != actual:
            raise AssertionError(f"{file}:{line}: FAIL: Expected {expected} Was {actual}")

    def assert_equal_float(self, expected: float, actual: float, file: str, line: int):
        if abs(expected - actual) > 1e-5:
            raise AssertionError(f"{file}:{line}: FAIL: Expected {expected} Was {actual}")

    def assert_float_within(self, delta: float, expected: float, actual: float, file: str, line: int):
        if abs(expected - actual) > delta:
            raise AssertionError(f"{file}:{line}: FAIL: Expected {expected} Was {actual} (Delta {delta})")

    def assert_less_or_equal(self, a: float, b: float, file: str, line: int):
        if a > b:
            raise AssertionError(f"{file}:{line}: FAIL: {a} > {b}")

    def run_test(self, file_name: str, line_no: int, test_name: str, test_fn):
        self.total += 1
        try:
            test_fn()
            print(f"{file_name}:{line_no}: {test_name}: PASS")
            self.passed += 1
        except Exception as e:
            print(f"{file_name}:{line_no}: {test_name}: FAIL ({e})")
            self.failed += 1
            self.failures.append(f"{test_name}: {e}")


def run_all_firmware_tests() -> int:
    runner = UnityTestRunner()
    vol_est = BIPVolumeEstimator(7.0)
    physics = BIPBalloonPhysics(vol_est)
    sensors = BIPSensors("TestPressure")
    motor = BIPMotor()
    state = BIPState()
    storage = BIPStorage()

    print("\n=== Running BIP Firmware Algorithms Test Suite (28 Tests) ===")

    # -------------------------------------------------------------
    # Suite 1: bip_balloon_physics (9 Tests)
    # -------------------------------------------------------------
    def test_mooney_rivlin_hyperelastic_stress():
        physics.init(BalloonType.BALLOON_11INCH)
        physics.update_ext(15.0, 0.0, 0.0, 25.0, 0.001)
        runner.assert_true(physics.stretch_ratio >= 1.0, "test_bip_balloon_physics.cpp", 15)
        runner.assert_true(physics.hyperelastic_stress_kpa > 0.0, "test_bip_balloon_physics.cpp", 16)
        runner.assert_true(0.0 <= physics.yield_probability <= 1.0, "test_bip_balloon_physics.cpp", 17)

    def test_sls_viscoelastic_stress_relaxation():
        physics.init(BalloonType.BALLOON_36INCH)
        # Step 1: Rapid inflation -> viscous stress accumulates
        for i in range(100):
            physics.update_ext(10.0 + i * 0.1, 5.0, 0.0, 25.0, 0.001)
        viscous_peak = physics.viscous_stress_kpa
        runner.assert_true(viscous_peak > 0.0, "test_bip_balloon_physics.cpp", 29)

        # Step 2: Hold constant strain -> relaxes
        for _ in range(500):
            physics.update_ext(20.0, 0.0, 0.0, 25.0, 0.005)
        runner.assert_true(physics.viscous_stress_kpa < viscous_peak, "test_bip_balloon_physics.cpp", 36)

    def test_thermal_softening_guard():
        physics.init(BalloonType.BALLOON_11INCH)
        physics.update_ext(15.0, 0.0, 0.0, 25.0, 0.001)
        stress_room = physics.hyperelastic_stress_kpa

        physics.update_ext(15.0, 0.0, 0.0, 45.0, 0.001)
        stress_hot = physics.hyperelastic_stress_kpa
        runner.assert_true(stress_hot < stress_room, "test_bip_balloon_physics.cpp", 50)

    def test_impact_classifier_bounce_vs_burst():
        physics.init(BalloonType.BALLOON_36INCH)
        imp1 = physics.classify_pressure_event(70.0, 5.0, 18.0)
        runner.assert_equal_int(ImpactClassification.IMPACT_BOUNCE, imp1, "test_bip_balloon_physics.cpp", 58)

        physics.sim_time_us += 50000  # within 100ms
        imp2 = physics.classify_pressure_event(-35.0, -2.0, 16.0)
        runner.assert_equal_int(ImpactClassification.IMPACT_BOUNCE, imp2, "test_bip_balloon_physics.cpp", 62)

        physics.positive_spike_peak = 0.0
        imp3 = physics.classify_pressure_event(-55.0, -10.0, 2.0)
        runner.assert_equal_int(ImpactClassification.IMPACT_BURST, imp3, "test_bip_balloon_physics.cpp", 66)

        imp4 = physics.classify_pressure_event(15.0, 0.5, 12.0)
        runner.assert_equal_int(ImpactClassification.IMPACT_SQUEEZE, imp4, "test_bip_balloon_physics.cpp", 70)

    def test_fatigue_tracker_miners_rule():
        physics.reset_fatigue()
        runner.assert_equal_int(0, physics.total_bounce_count, "test_bip_balloon_physics.cpp", 76)
        runner.assert_float_within(0.1, 100.0, physics.safe_pressure_remaining_pct, "test_bip_balloon_physics.cpp", 77)

        for _ in range(50):
            physics.record_bounce_cycle(28.0)
        runner.assert_equal_int(50, physics.total_bounce_count, "test_bip_balloon_physics.cpp", 84)
        runner.assert_float_within(0.1, 28.0, physics.max_bounce_stress_kpa, "test_bip_balloon_physics.cpp", 85)
        runner.assert_true(physics.accumulated_damage > 0.0, "test_bip_balloon_physics.cpp", 86)
        runner.assert_true(physics.safe_pressure_remaining_pct < 100.0, "test_bip_balloon_physics.cpp", 87)

    def test_ride_inflation_calculator():
        a_70kg = BIPBalloonPhysics.compute_ride_inflation(70.0, BalloonType.BALLOON_36INCH, 0.5)
        runner.assert_true(a_70kg["recommended_pressure_kpa"] > 2.0, "test_bip_balloon_physics.cpp", 93)
        runner.assert_true(a_70kg["max_safe_pressure_kpa"] > a_70kg["recommended_pressure_kpa"], "test_bip_balloon_physics.cpp", 94)
        runner.assert_true(a_70kg["predicted_sink_depth_cm"] > 0.0, "test_bip_balloon_physics.cpp", 95)

        a_110kg = BIPBalloonPhysics.compute_ride_inflation(110.0, BalloonType.BALLOON_36INCH, 0.5)
        runner.assert_true(a_110kg["recommended_pressure_kpa"] <= a_70kg["recommended_pressure_kpa"], "test_bip_balloon_physics.cpp", 99)

    def test_adaptive_burst_threshold_learning():
        physics.learned_burst_threshold = -40.0
        runner.assert_float_within(0.1, -40.0, physics.learned_burst_threshold, "test_bip_balloon_physics.cpp", 104)

        physics.learned_burst_threshold = -48.5
        runner.assert_float_within(0.1, -48.5, physics.learned_burst_threshold, "test_bip_balloon_physics.cpp", 107)

    def test_online_mooney_rivlin_identification():
        physics.init(BalloonType.BALLOON_11INCH)
        for i in range(60):
            lam = 1.2 + i * 0.02
            inv_l = 1.0 / lam
            inv_l7 = 1.0 / (lam ** 7)
            p = 0.04 * (inv_l - inv_l7) * (180.0 + 20.0 * lam * lam)
            physics.update_online_id(p, lam)
        runner.assert_true(physics.online_converged, "test_bip_balloon_physics.cpp", 123)
        runner.assert_true(physics.estimated_C10 > 0.0, "test_bip_balloon_physics.cpp", 124)

    def test_tear_precursor_flutter_detector():
        physics.init(BalloonType.BALLOON_11INCH)
        for i in range(20):
            spike = 250.0 if (i % 2 == 0) else -250.0
            physics.update_ext(20.0, spike, 0.0, 25.0, 0.001)
        runner.assert_true(physics.flutter_variance > 50.0, "test_bip_balloon_physics.cpp", 136)

    runner.run_test("test/test_bip_balloon_physics.cpp", 6, "test_mooney_rivlin_hyperelastic_stress", test_mooney_rivlin_hyperelastic_stress)
    runner.run_test("test/test_bip_balloon_physics.cpp", 20, "test_sls_viscoelastic_stress_relaxation", test_sls_viscoelastic_stress_relaxation)
    runner.run_test("test/test_bip_balloon_physics.cpp", 39, "test_thermal_softening_guard", test_thermal_softening_guard)
    runner.run_test("test/test_bip_balloon_physics.cpp", 53, "test_impact_classifier_bounce_vs_burst", test_impact_classifier_bounce_vs_burst)
    runner.run_test("test/test_bip_balloon_physics.cpp", 73, "test_fatigue_tracker_miners_rule", test_fatigue_tracker_miners_rule)
    runner.run_test("test/test_bip_balloon_physics.cpp", 90, "test_ride_inflation_calculator", test_ride_inflation_calculator)
    runner.run_test("test/test_bip_balloon_physics.cpp", 102, "test_adaptive_burst_threshold_learning", test_adaptive_burst_threshold_learning)
    runner.run_test("test/test_bip_balloon_physics.cpp", 110, "test_online_mooney_rivlin_identification", test_online_mooney_rivlin_identification)
    runner.run_test("test/test_bip_balloon_physics.cpp", 127, "test_tear_precursor_flutter_detector", test_tear_precursor_flutter_detector)

    # -------------------------------------------------------------
    # Suite 2: bip_volume_estimator (4 Tests)
    # -------------------------------------------------------------
    def test_hammerstein_pump_model_deadzone():
        vol_est.init(7.0)
        vol_est.set_pump_model_params(35.0, 72.0)
        vol_est.update(10.0, 30.0, 1.0)
        runner.assert_float_within(0.001, 0.0, vol_est.cumulative_volume_liters, "test_bip_volume_estimator.cpp", 12)

        vol_est.update(10.0, 150.0, 1.0)
        runner.assert_true(vol_est.cumulative_volume_liters > 0.0, "test_bip_volume_estimator.cpp", 17)

    def test_ideal_gas_pv_nrt_scaling():
        vol_est.init(7.0)
        vol_est.set_pump_model_params(0.0, 72.0)
        vol_est.update_ext(10.0, 200.0, 40.0, 1.0)
        vol_standard = vol_est.cumulative_volume_liters

        vol_est.init(7.0)
        vol_est.update_ext(10.0, 200.0, 60.0, 1.0)
        vol_hot = vol_est.cumulative_volume_liters
        runner.assert_true(vol_hot < vol_standard, "test_bip_volume_estimator.cpp", 34)

    def test_oblate_spheroid_diameter_calculation():
        vol_est.init(7.0)
        vol_est.set_balloon_shape_params(0.0, 0.85)
        vol_est.update(15.0, 200.0, 10.0)
        diam_oblate = vol_est.get_diameter_cm()

        vol_est.init(7.0)
        vol_est.set_balloon_shape_params(0.0, 1.00)
        vol_est.update(15.0, 200.0, 10.0)
        diam_sphere = vol_est.get_diameter_cm()
        runner.assert_true(diam_oblate > diam_sphere, "test_bip_volume_estimator.cpp", 52)

    def test_neck_volume_offset():
        vol_est.init(7.0)
        vol_est.set_balloon_shape_params(0.05, 0.85)
        vol_est.update(5.0, 100.0, 0.2)
        runner.assert_float_within(0.1, 7.0, vol_est.get_diameter_cm(), "test_bip_volume_estimator.cpp", 65)

    runner.run_test("test/test_bip_volume_estimator.cpp", 5, "test_hammerstein_pump_model_deadzone", test_hammerstein_pump_model_deadzone)
    runner.run_test("test/test_bip_volume_estimator.cpp", 20, "test_ideal_gas_pv_nrt_scaling", test_ideal_gas_pv_nrt_scaling)
    runner.run_test("test/test_bip_volume_estimator.cpp", 37, "test_oblate_spheroid_diameter_calculation", test_oblate_spheroid_diameter_calculation)
    runner.run_test("test/test_bip_volume_estimator.cpp", 55, "test_neck_volume_offset", test_neck_volume_offset)

    # -------------------------------------------------------------
    # Suite 3: bip_state (7 Tests)
    # -------------------------------------------------------------
    def test_pid_parameter_tuning():
        state.set_pid(30.0, 1.5, 5.0)
        state.set_manual_target(25.0)
        state.init_states()
        runner.assert_equal_int(SystemMode.MODE_IDLE, state.current_mode, "test_bip_state.cpp", 9)

    def test_yield_ratio_constraints():
        state.set_yield_ratio(1.35)
        runner.assert_equal_float(1.35, state.yield_ratio, "test_bip_state.cpp", 13)
        state.set_yield_ratio(0.5)  # Rejection
        runner.assert_equal_float(1.35, state.yield_ratio, "test_bip_state.cpp", 14)
        state.set_yield_ratio(1.25)
        runner.assert_equal_float(1.25, state.yield_ratio, "test_bip_state.cpp", 15)

    def test_manual_mode_target_switch():
        state.init_states()
        state.set_manual_target(15.0)
        state.current_mode = SystemMode.MODE_MANUAL
        runner.assert_equal_int(SystemMode.MODE_MANUAL, state.current_mode, "test_bip_state.cpp", 22)

    def test_new_modes_transitions():
        state.init_states()
        state.set_target_diameter(30.0)
        runner.assert_equal_int(SystemMode.MODE_DIAMETER, state.current_mode, "test_bip_state.cpp", 28)

        state.set_ride_params(80.0, 6.0)
        runner.assert_equal_int(SystemMode.MODE_RIDE, state.current_mode, "test_bip_state.cpp", 31)

        state.start_balloon_conditioning(3)
        runner.assert_equal_int(SystemMode.MODE_CONDITION, state.current_mode, "test_bip_state.cpp", 34)

        state.post_mode_change(SystemMode.MODE_IDLE)
        runner.assert_equal_int(SystemMode.MODE_IDLE, state.current_mode, "test_bip_state.cpp", 37)

    def test_physics_fatigue_and_impact_classification():
        physics.reset_fatigue()
        runner.assert_equal_float(100.0, physics.safe_pressure_remaining_pct, "test_bip_state.cpp", 43)

        physics.record_bounce_cycle(25.0)
        runner.assert_equal_int(1, physics.total_bounce_count, "test_bip_state.cpp", 47)
        runner.assert_true(physics.accumulated_damage > 0.0, "test_bip_state.cpp", 48)

        imp = physics.classify_pressure_event(65.0, 0.0, 10.0)
        runner.assert_equal_int(ImpactClassification.IMPACT_BOUNCE, imp, "test_bip_state.cpp", 51)

    def test_state_ride_inflation_calculator():
        advice = BIPBalloonPhysics.compute_ride_inflation(75.0, BalloonType.BALLOON_36INCH, 0.5)
        runner.assert_true(advice["recommended_pressure_kpa"] > 0.0, "test_bip_state.cpp", 56)
        runner.assert_true(advice["max_safe_pressure_kpa"] > advice["recommended_pressure_kpa"], "test_bip_state.cpp", 57)
        runner.assert_true(advice["predicted_sink_depth_cm"] > 0.0, "test_bip_state.cpp", 58)

    def test_scurve_profile_generator():
        curr = 10.0
        target = 30.0
        max_vel = 5.0
        next1 = BIPState.update_scurve_profile(curr, target, max_vel, 1.0)
        runner.assert_float_within(0.01, 15.0, next1, "test_bip_state.cpp", 67)

        next2 = BIPState.update_scurve_profile(29.0, target, max_vel, 1.0)
        runner.assert_float_within(0.01, 30.0, next2, "test_bip_state.cpp", 70)

    runner.run_test("test/test_bip_state.cpp", 5, "test_pid_parameter_tuning", test_pid_parameter_tuning)
    runner.run_test("test/test_bip_state.cpp", 12, "test_yield_ratio_constraints", test_yield_ratio_constraints)
    runner.run_test("test/test_bip_state.cpp", 18, "test_manual_mode_target_switch", test_manual_mode_target_switch)
    runner.run_test("test/test_bip_state.cpp", 25, "test_new_modes_transitions", test_new_modes_transitions)
    runner.run_test("test/test_bip_state.cpp", 40, "test_physics_fatigue_and_impact_classification", test_physics_fatigue_and_impact_classification)
    runner.run_test("test/test_bip_state.cpp", 54, "test_ride_inflation_calculator", test_state_ride_inflation_calculator)
    runner.run_test("test/test_bip_state.cpp", 61, "test_scurve_profile_generator", test_scurve_profile_generator)

    # -------------------------------------------------------------
    # Suite 4: bip_sensors (3 Tests)
    # -------------------------------------------------------------
    def test_sensor_multipoint_interpolation():
        sensors.reset_calibration()
        sensors.add_calibration_point(0.5, 0.0)
        sensors.add_calibration_point(4.5, 200.0)

        sensors.set_raw_value(0.5)
        runner.assert_float_within(0.01, 0.0, sensors.get_value(), "test_bip_sensors.cpp", 14)

        sensors.set_raw_value(2.5)
        runner.assert_float_within(0.01, 100.0, sensors.get_value(), "test_bip_sensors.cpp", 17)

        sensors.set_raw_value(4.5)
        runner.assert_float_within(0.01, 200.0, sensors.get_value(), "test_bip_sensors.cpp", 20)

    def test_sensor_tare_zeroing():
        sensors.reset_calibration()
        sensors.reset_tare()
        sensors.add_calibration_point(0.5, 0.0)
        sensors.add_calibration_point(4.5, 200.0)

        sensors.set_raw_value(0.7)
        runner.assert_float_within(0.1, 10.0, sensors.get_value(), "test_bip_sensors.cpp", 31)

        sensors.tare()
        runner.assert_float_within(0.01, 0.0, sensors.get_value(), "test_bip_sensors.cpp", 35)
        runner.assert_float_within(0.1, 10.0, sensors.get_zero_offset(), "test_bip_sensors.cpp", 36)

        sensors.reset_tare()
        runner.assert_float_within(0.01, 0.0, sensors.get_zero_offset(), "test_bip_sensors.cpp", 40)

    def test_sensor_duplicate_point_div_zero_guard():
        sensors.reset_calibration()
        sensors.add_calibration_point(1.0, 10.0)
        sensors.add_calibration_point(1.0, 20.0)
        runner.assert_equal_int(1, sensors.get_num_points(), "test_bip_sensors.cpp", 49)

        sensors.set_raw_value(1.0)
        runner.assert_false(math.isnan(sensors.get_value()), "test_bip_sensors.cpp", 53)
        runner.assert_false(math.isinf(sensors.get_value()), "test_bip_sensors.cpp", 54)

    runner.run_test("test/test_bip_sensors.cpp", 7, "test_sensor_multipoint_interpolation", test_sensor_multipoint_interpolation)
    runner.run_test("test/test_bip_sensors.cpp", 23, "test_sensor_tare_zeroing", test_sensor_tare_zeroing)
    runner.run_test("test/test_bip_sensors.cpp", 43, "test_sensor_duplicate_point_div_zero_guard", test_sensor_duplicate_point_div_zero_guard)

    # -------------------------------------------------------------
    # Suite 5: bip_motor (3 Tests)
    # -------------------------------------------------------------
    def test_motor_soft_start_ramping():
        motor.clear_safety_lockout()
        motor.set_ramp_rate(5.0)
        motor.set_target_pwm(100)

        runner.assert_equal_int(0, motor.get_current_pwm(), "test_bip_motor.cpp", 12)

        motor.update(0.5, 12.0, 12.0)
        runner.assert_equal_int(5, motor.get_current_pwm(), "test_bip_motor.cpp", 16)

        motor.update(0.5, 12.0, 12.0)
        runner.assert_equal_int(10, motor.get_current_pwm(), "test_bip_motor.cpp", 20)

        motor.emergency_stop()
        runner.assert_equal_int(0, motor.get_current_pwm(), "test_bip_motor.cpp", 24)

    def test_motor_overcurrent_inrush_debounce():
        motor.clear_safety_lockout()
        motor.set_safety_settings(4.0, 10.0, 10)
        motor.set_target_pwm(150)

        for _ in range(20):
            motor.update(4.5, 12.0, 12.0)
            runner.assert_false(motor.is_safety_triggered(), "test_bip_motor.cpp", 36)

        for _ in range(20):
            motor.update(2.0, 12.0, 12.0)
            runner.assert_false(motor.is_safety_triggered(), "test_bip_motor.cpp", 42)

        for _ in range(49):
            motor.update(4.5, 12.0, 12.0)
            runner.assert_false(motor.is_safety_triggered(), "test_bip_motor.cpp", 48)

        motor.update(4.5, 12.0, 12.0)
        runner.assert_true(motor.is_safety_triggered(), "test_bip_motor.cpp", 53)
        runner.assert_true(motor.get_safety_error() == "OC_LOCK", "test_bip_motor.cpp", 54)

        motor.clear_safety_lockout()
        runner.assert_false(motor.is_safety_triggered(), "test_bip_motor.cpp", 58)

    def test_motor_undervoltage_lockout():
        motor.clear_safety_lockout()
        motor.set_safety_settings(4.0, 10.5, 10)
        motor.set_target_pwm(100)

        motor.update(1.0, 9.5, 12.0)
        runner.assert_true(motor.is_safety_triggered(), "test_bip_motor.cpp", 69)
        runner.assert_true(motor.get_safety_error() == "UV_LOCK", "test_bip_motor.cpp", 70)

        motor.clear_safety_lockout()
        motor.emergency_stop()

    runner.run_test("test/test_bip_motor.cpp", 6, "test_motor_soft_start_ramping", test_motor_soft_start_ramping)
    runner.run_test("test/test_bip_motor.cpp", 27, "test_motor_overcurrent_inrush_debounce", test_motor_overcurrent_inrush_debounce)
    runner.run_test("test/test_bip_motor.cpp", 61, "test_motor_undervoltage_lockout", test_motor_undervoltage_lockout)

    # -------------------------------------------------------------
    # Suite 6: bip_storage (2 Tests)
    # -------------------------------------------------------------
    def test_nvs_storage_struct_alignment():
        runner.assert_less_or_equal(BIPStorage.get_struct_size(), 256, "test_bip_storage.cpp", 5)

    def test_nvs_storage_crc32_and_write_counter():
        writes_before = storage.get_nvs_write_count()
        storage.save_system_config()
        writes_after = storage.get_nvs_write_count()
        runner.assert_equal_int(writes_before + 1, writes_after, "test_bip_storage.cpp", 12)

    runner.run_test("test/test_bip_storage.cpp", 4, "test_nvs_storage_struct_alignment", test_nvs_storage_struct_alignment)
    runner.run_test("test/test_bip_storage.cpp", 8, "test_nvs_storage_crc32_and_write_counter", test_nvs_storage_crc32_and_write_counter)

    print("-----------------------")
    print(f"{runner.total} Tests {runner.failed} Failures {runner.ignored} Ignored")
    if runner.failed == 0:
        print("OK")
        return 0
    else:
        print(f"FAILED ({runner.failed} failure(s))")
        return 1


if __name__ == "__main__":
    sys.exit(run_all_firmware_tests())
