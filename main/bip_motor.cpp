#include "bip_motor.h"
#include "esp_log.h"
#include <algorithm>

static const char *TAG = "BIP_MOTOR";

BIP_Motor pumpMotor(PIN_MOTOR_PWM);

BIP_Motor::BIP_Motor(gpio_num_t pin) {
    _pin = pin;
    _targetPWM = 0;
    _currentPWM_f = 0;
    _safetyTriggered = false;
    _rampRate = 2.0f;
    _sagPwmLimit = 255;
    _idleVoltsSnapshot = 0;
    _ocOverCount = 0;
    _errorMsg = "";
    
    _safety.maxCurrent = SAFETY_OC_DEFAULT;
    _safety.minVoltage = SAFETY_UV_DEFAULT;
    _safety.sagLimitPercent = SAFETY_SAG_DEFAULT;
}

esp_err_t BIP_Motor::begin() {
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    timer_conf.duty_resolution = PWM_RESOLUTION;
    timer_conf.timer_num = PWM_TIMER;
    timer_conf.freq_hz = MOTOR_PWM_FREQ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) return ret;

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num = _pin;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.channel = MOTOR_PWM_CHANNEL;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = PWM_TIMER;
    ch_conf.duty = 0;
    ch_conf.hpoint = 0;
    ret = ledc_channel_config(&ch_conf);
    
    ESP_LOGI(TAG, "Motor 25kHz LEDC PWM driver initialized");
    return ret;
}

void BIP_Motor::setTargetPWM(uint8_t pwm) {
    if (_safetyTriggered && pwm > 0) return;
    if (_targetPWM == 0 && pwm > 0) {
        _sagPwmLimit = 255;
        _idleVoltsSnapshot = 0;
    }
    _targetPWM = pwm;
}

void BIP_Motor::setRampRate(float rate) {
    if (rate < 0.1f) rate = 0.1f;
    if (rate > 20.0f) rate = 20.0f;
    _rampRate = rate;
}

void BIP_Motor::emergencyStop() {
    _targetPWM = 0;
    _currentPWM_f = 0;
    _sagPwmLimit = 255;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL, 0);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL);
}

void BIP_Motor::setSafetySettings(SafetySettings settings) {
    _safety = settings;
}

SafetySettings BIP_Motor::getSafetySettings() {
    return _safety;
}

bool BIP_Motor::isSafetyTriggered() {
    return _safetyTriggered;
}

void BIP_Motor::clearSafetyLockout() {
    _safetyTriggered = false;
    _errorMsg = "";
    _ocOverCount = 0;
}

const char* BIP_Motor::getSafetyError() {
    return _errorMsg;
}

uint8_t BIP_Motor::getCurrentPWM() {
    return (uint8_t)_currentPWM_f;
}

void BIP_Motor::update(float currentAmps, float inputVolts, float idleVolts) {
    if (_safetyTriggered) {
        emergencyStop();
        return;
    }
    
    if (_targetPWM == 0 && inputVolts > 5.0f) {
        _idleVoltsSnapshot = inputVolts;
    }
    if (_idleVoltsSnapshot < 5.0f) _idleVoltsSnapshot = (idleVolts > 5.0f) ? idleVolts : 12.0f;

    if (inputVolts < _safety.minVoltage && inputVolts > 1.0f) {
        _safetyTriggered = true;
        _errorMsg = "UV_LOCK";
        emergencyStop();
        return;
    }
    
    if (currentAmps > _safety.maxCurrent) {
        _ocOverCount++;
        if (_ocOverCount >= 3) {
            _safetyTriggered = true;
            _errorMsg = "OC_LOCK";
            emergencyStop();
            return;
        }
    } else {
        if (_ocOverCount > 0) _ocOverCount--;
    }
    
    if (_targetPWM > 0) {
        float sagThreshold = _idleVoltsSnapshot * (1.0f - (_safety.sagLimitPercent / 100.0f));
        if (inputVolts < sagThreshold) {
            if (_sagPwmLimit > 0) _sagPwmLimit--;
        }
    } else {
        _sagPwmLimit = 255;
    }
    
    uint8_t effectiveTarget = std::min(_targetPWM, _sagPwmLimit);

    if (_currentPWM_f < effectiveTarget) {
        _currentPWM_f += _rampRate;
        if (_currentPWM_f > effectiveTarget) _currentPWM_f = effectiveTarget;
    } else if (_currentPWM_f > effectiveTarget) {
        _currentPWM_f -= _rampRate * 2.0f;
        if (_currentPWM_f < effectiveTarget) _currentPWM_f = effectiveTarget;
    }
    
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL, (uint32_t)_currentPWM_f);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, MOTOR_PWM_CHANNEL);
}

esp_err_t init_solenoid_valve() {
    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num = PIN_SOLENOID_VALVE;
    ch_conf.speed_mode = LEDC_HIGH_SPEED_MODE;
    ch_conf.channel = VALVE_PWM_CHANNEL;
    ch_conf.intr_type = LEDC_INTR_DISABLE;
    ch_conf.timer_sel = PWM_TIMER;
    ch_conf.duty = 0;
    ch_conf.hpoint = 0;
    return ledc_channel_config(&ch_conf);
}

void write_solenoid_pwm(uint8_t pwm) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, VALVE_PWM_CHANNEL, pwm);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, VALVE_PWM_CHANNEL);
}
