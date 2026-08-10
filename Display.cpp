#include "Display.h"

// Address 0x27, 16 chars, 2 lines
Display lcdDisplay;

Display::Display() : _lcd(0x27, 16, 2) {
    _lastUpdate = 0;
}

void Display::begin() {
    _lcd.init();
    _lcd.backlight();
}

void Display::clear() {
    _lcd.clear();
}

void Display::showBoot() {
    _lcd.setCursor(0,0);
    _lcd.print("BalloonTester v1");
    _lcd.setCursor(0,1);
    _lcd.print("Init Sensors... ");
}

void Display::showManual(float pressure, float peak, int pwm, float volts, float amps, String status) {
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "P:%-5.1f PWM:%-3d", pressure, pwm);
    snprintf(line1, sizeof(line1), "%-4.1fV %-4.1fA %-4s", volts, amps, status.c_str());
    _lcd.setCursor(0,0);
    _lcd.print(line0);
    _lcd.setCursor(0,1);
    _lcd.print(line1);
}

void Display::showSmart(float pressure, float currentPeak, float firstYield, float target, String status) {
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "P:%-4.0f Yld:%-5.0f", pressure, firstYield);
    snprintf(line1, sizeof(line1), "T:%-4.0f S:%-6s", target, status.c_str());
    _lcd.setCursor(0,0);
    _lcd.print(line0);
    _lcd.setCursor(0,1);
    _lcd.print(line1);
}

void Display::showBurst(float pressure, float peak, float amps, String status) {
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "P: %-5.1f kPa   ", pressure);
    snprintf(line1, sizeof(line1), "Pk:%-5.1f %-5s", peak, status.c_str());
    _lcd.setCursor(0,0);
    _lcd.print(line0);
    _lcd.setCursor(0,1);
    _lcd.print(line1);
}

void Display::showBurstPopped(float peak, float amps) {
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "** POPPED! **   ");
    snprintf(line1, sizeof(line1), "PEAK:%-5.1fkPa   ", peak);
    _lcd.setCursor(0,0);
    _lcd.print(line0);
    _lcd.setCursor(0,1);
    _lcd.print(line1);
}

void Display::showError(String err1, String err2) {
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "%-16s", err1.c_str());
    snprintf(line1, sizeof(line1), "%-16s", err2.c_str());
    _lcd.setCursor(0,0);
    _lcd.print(line0);
    _lcd.setCursor(0,1);
    _lcd.print(line1);
}

void Display::showCalMenu(int itemIndex, float currentVal, float savedVal) {
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "Calib Menu: %-2d  ", itemIndex);
    snprintf(line1, sizeof(line1), "%-5.1f -> %-5.1f", currentVal, savedVal);
    _lcd.setCursor(0,0);
    _lcd.print(line0);
    _lcd.setCursor(0,1);
    _lcd.print(line1);
}

void Display::showAutoCal(float currentP, float refP, bool done) {
    char line0[17];
    char line1[17];
    snprintf(line0, sizeof(line0), "AutoCal: %-7s", done ? "DONE!" : "RUN...");
    snprintf(line1, sizeof(line1), "%-5.1f vs %-5.1f", currentP, refP);
    _lcd.setCursor(0,0);
    _lcd.print(line0);
    _lcd.setCursor(0,1);
    _lcd.print(line1);
}
