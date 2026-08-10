#include "Motor.h"

Motor pumpMotor(PIN_MOTOR_PWM);

// Helper for cross-version ESP32 LEDC PWM output
static void writeMotorPWM(uint8_t pin, uint8_t val) {
#ifdef ESP32
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(pin, val);
    #else
        ledcWrite(MOTOR_PWM_CHANNEL, val);
    #endif
#else
    analogWrite(pin, val);
#endif
}

void initMotor() {
    pumpMotor.begin();
}

// ==========================================
// Motor Class Implementation
// ==========================================

Motor::Motor(uint8_t pin) {
    _pin = pin;
    _targetPWM = 0;
    _currentPWM_f = 0;
    _safetyTriggered = false;
    _rampRate = 2.0f; // Approx 1.2s to full speed
    _sagPwmLimit = 255;
    _idleVoltsSnapshot = 0;
    _ocOverCount = 0;
}

void Motor::begin() {
#ifdef ESP32
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcAttach(_pin, MOTOR_PWM_FREQ, PWM_RESOLUTION);
    #else
        ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(_pin, MOTOR_PWM_CHANNEL);
    #endif
    writeMotorPWM(_pin, 0);
#else
    pinMode(_pin, OUTPUT);
    analogWrite(_pin, 0);
#endif
}

void Motor::setTargetPWM(uint8_t pwm) {
    if (_safetyTriggered && pwm > 0) return; // Lockout
    
    // Reset Sag Limit if starting from 0
    if (_targetPWM == 0 && pwm > 0) {
        _sagPwmLimit = 255;
        _idleVoltsSnapshot = 0; 
    }
    
    _targetPWM = pwm;
}

void Motor::setRampRate(float rate) {
    if (rate < 0.1f) rate = 0.1f;
    if (rate > 20.0f) rate = 20.0f;
    _rampRate = rate;
}

void Motor::emergencyStop() {
    _targetPWM = 0;
    _currentPWM_f = 0;
    _sagPwmLimit = 255; 
    writeMotorPWM(_pin, 0);
}

void Motor::setSafetySettings(SafetySettings settings) {
    _safety = settings;
}

SafetySettings Motor::getSafetySettings() {
    return _safety;
}

bool Motor::isSafetyTriggered() {
    return _safetyTriggered;
}

void Motor::clearSafetyLockout() {
    _safetyTriggered = false;
    _errorMsg = "";
    _ocOverCount = 0;
}

String Motor::getSafetyError() {
    return _errorMsg;
}

uint8_t Motor::getCurrentPWM() {
    return (uint8_t)_currentPWM_f;
}

void Motor::update(float currentAmps, float inputVolts, float idleVolts) {
    // 1. Safety Checks
    if (_safetyTriggered) {
        emergencyStop();
        return;
    }
    
    // Capture Idle Volts once when motor is off
    if (_targetPWM == 0 && inputVolts > 5.0f) {
        _idleVoltsSnapshot = inputVolts;
    }
    if (_idleVoltsSnapshot < 5.0f) _idleVoltsSnapshot = (idleVolts > 5.0f) ? idleVolts : 12.0f;

    // Undervoltage check
    if (inputVolts < _safety.minVoltage && inputVolts > 1.0f) { 
        _safetyTriggered = true;
        _errorMsg = "UV_LOCK";
        emergencyStop();
        return;
    }
    
    // Overcurrent check with 3-sample inrush debounce
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
    
    // ----------------------------
    // Sag Protection (Ratcheting)
    // ----------------------------
    if (_targetPWM > 0) {
        float sagThreshold = _idleVoltsSnapshot * (1.0f - (_safety.sagLimitPercent / 100.0f));
        
        // If voltage is sagging below threshold
        if (inputVolts < sagThreshold) {
            if (_sagPwmLimit > 0) {
                _sagPwmLimit--; 
            }
        }
    } else {
        _sagPwmLimit = 255; // Reset when stopped
    }
    
    // Apply Sag Limit to Target
    uint8_t effectiveTarget = min(_targetPWM, _sagPwmLimit);

    // 2. Soft Start Ramping
    if (_currentPWM_f < effectiveTarget) {
        _currentPWM_f += _rampRate;
        if (_currentPWM_f > effectiveTarget) _currentPWM_f = effectiveTarget;
    } else if (_currentPWM_f > effectiveTarget) {
        _currentPWM_f -= _rampRate * 2.0f; // Decelerate faster
        if (_currentPWM_f < effectiveTarget) _currentPWM_f = effectiveTarget;
    }
    
    writeMotorPWM(_pin, (uint8_t)_currentPWM_f);
}
