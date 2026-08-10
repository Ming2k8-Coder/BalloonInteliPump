#ifndef BIP_MOTOR_H
#define BIP_MOTOR_H

#include "bip_config.h"
#include "esp_err.h"

typedef struct {
    float maxCurrent;
    float minVoltage;
    uint8_t sagLimitPercent;
} SafetySettings;

class BIP_Motor {
public:
    BIP_Motor(gpio_num_t pin);
    esp_err_t begin();
    
    void setTargetPWM(uint8_t pwm);
    void emergencyStop();
    void update(float currentAmps, float inputVolts, float idleVolts);
    
    void setSafetySettings(SafetySettings settings);
    SafetySettings getSafetySettings();
    bool isSafetyTriggered();
    void clearSafetyLockout();
    const char* getSafetyError();
    
    void setRampRate(float rate);
    uint8_t getCurrentPWM();

private:
    gpio_num_t _pin;
    uint8_t _targetPWM;
    float _currentPWM_f;
    uint8_t _sagPwmLimit;
    float _idleVoltsSnapshot;
    uint8_t _ocOverCount;
    
    SafetySettings _safety;
    bool _safetyTriggered;
    const char* _errorMsg;
    float _rampRate;
};

extern BIP_Motor pumpMotor;
esp_err_t init_solenoid_valve();
void write_solenoid_pwm(uint8_t pwm);

#endif // BIP_MOTOR_H
