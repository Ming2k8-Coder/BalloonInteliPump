#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Config.h"

class Display {
public:
    Display();
    void begin();
    void updateLegacy(); // Old
    
    // Screens
    void showBoot();
    void showManual(float pressure, float peak, int pwm, float volts, float amps, String status);
    void showSmart(float pressure, float currentPeak, float firstYield, float target, String status);
    void showBurst(float pressure, float peak, float amps, String status);
    void showBurstPopped(float peak, float amps);
    void showCalMenu(int itemIndex, float currentVal, float savedVal);
    void showAutoCal(float currentP, float refP, bool done);
    void showError(String err1, String err2);
    
    // Low level
    void clear();
    
private:
    LiquidCrystal_I2C _lcd;
    unsigned long _lastUpdate;
};

extern Display lcdDisplay;

#endif // DISPLAY_H
