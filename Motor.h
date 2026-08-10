#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Config.h"

struct SafetySettings {
    float maxCurrent;
    float minVoltage;
    uint8_t sagLimitPercent;
};

class Motor {
public:
    Motor(uint8_t pin);
    void begin();
    
    // Core Control
    void setTargetPWM(uint8_t pwm); // 0-255
    void emergencyStop(); // Immediate 0
    void update(float currentAmps, float inputVolts, float idleVolts);
    
    // Safety Configuration
    void setSafetySettings(SafetySettings settings);
    SafetySettings getSafetySettings();
    bool isSafetyTriggered();
    void clearSafetyLockout();
    String getSafetyError(); // Returns "UV", "OC", "SAG" etc.

    // Ramping & Status
    void setRampRate(float rate);
    uint8_t getCurrentPWM();

private:
    uint8_t _pin;
    uint8_t _targetPWM;
    float _currentPWM_f; // Float for smooth ramping
    
    // Sag Protection & Overcurrent Debouncing
    uint8_t _sagPwmLimit; // Ratcheting limit (can only go down during run)
    float _idleVoltsSnapshot;
    uint8_t _ocOverCount; // Overcurrent debounce sample counter

    SafetySettings _safety;
    bool _safetyTriggered;
    String _errorMsg;
    
    // Soft Start Ramping
    float _rampRate; // PWM units per loop
};

extern Motor pumpMotor;
void initMotor();

#endif // MOTOR_H
