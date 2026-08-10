#include "StateFunctions.h"
#include "Sensors.h"
#include "Motor.h"
#include "Input.h"
#include "Display.h"
#include "Storage.h"

// Globals
SystemMode currentMode = MODE_IDLE;
String errorLine1 = "";
String errorLine2 = "";

// Smart Mode Vars
enum SmartPhase {
    SMART_IDLE,
    SMART_INFLATING_YIELD, // Looking for Pk1
    SMART_INFLATING_TARGET, // Going to Pk1 * Ratio
    SMART_FINISHED
};
SmartPhase smartState = SMART_IDLE;
float userYieldRatio = 1.20f; // Default 120%
float detectedYieldPressure = 0;
float peakPressureTracker = 0;
unsigned long startTime = 0;

// Manual Mode Vars
int manualPWM = 0;
bool manualTargetMode = false;
float manualTargetPressure = 20.0f;

// PID Controller Settings & States
float pid_Kp = 25.0f;  // Proportional gain
float pid_Ki = 1.2f;   // Integral gain
float pid_Kd = 4.0f;   // Derivative gain
float pid_integral = 0.0f;
float pid_lastError = 0.0f;
unsigned long pid_lastTime = 0;

// Smart Mode dP/dt Sliding Window History
#define SLOPE_WINDOW_SIZE 10
float pressureHistory[SLOPE_WINDOW_SIZE] = {0};
unsigned long timeHistory[SLOPE_WINDOW_SIZE] = {0};
int historyIndex = 0;
bool historyFull = false;
int flatCount = 0; // Fix: moved from static inside case

void resetHistory() {
    memset(pressureHistory, 0, sizeof(pressureHistory));
    memset(timeHistory, 0, sizeof(timeHistory));
    historyIndex = 0;
    historyFull = false;
}

void addHistoryPoint(float p) {
    pressureHistory[historyIndex] = p;
    timeHistory[historyIndex] = micros(); // Fix: use micros() for better precision
    historyIndex = (historyIndex + 1) % SLOPE_WINDOW_SIZE;
    if (historyIndex == 0) historyFull = true;
}

float calculateSlope() {
    if (!historyFull) return 1.0f; // Assume positive rising slope during fill start
    
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    int n = SLOPE_WINDOW_SIZE;
    unsigned long baseTime = timeHistory[historyIndex]; // oldest is at historyIndex
    
    for (int i = 0; i < n; i++) {
        int idx = (historyIndex + i) % n;
        float x = (timeHistory[idx] - baseTime) / 1000000.0f; // time in seconds (from micros)
        float y = pressureHistory[idx];
        
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumXX += x * x;
    }
    
    float denominator = (n * sumXX - sumX * sumX);
    if (abs(denominator) < 0.0001f) return 0.0f;
    
    return (n * sumXY - sumX * sumY) / denominator;
}

// Burst Mode Vars
float burstPeak = 0;
bool burstPopped = false;

void initStates() {
    currentMode = MODE_IDLE;
    burstPopped = false;
    burstPeak = 0;
}

void triggerError(String e1, String e2) {
    pumpMotor.emergencyStop();
    errorLine1 = e1;
    errorLine2 = e2;
    currentMode = MODE_ERROR;
}

void setYieldRatio(float ratio) {
    if (ratio >= 1.0) userYieldRatio = ratio;
}

void setManualTarget(float pressure) {
    manualTargetPressure = pressure;
    manualTargetMode = true; // Auto-enable target mode if set via Serial
    pid_lastTime = 0;
    pid_integral = 0;
}

void setManualMode(bool targetMode) {
    manualTargetMode = targetMode;
    pid_lastTime = 0;
    pid_integral = 0;
}

void setPID(float kp, float ki, float kd) {
    pid_Kp = kp;
    pid_Ki = ki;
    pid_Kd = kd;
    pid_integral = 0;
    pid_lastError = 0;
}

void updateStateMachine() {
    switch (currentMode) {
        case MODE_IDLE: updateIdle(); break;
        case MODE_MANUAL: updateManual(); break;
        case MODE_SMART: updateSmart(); break;
        case MODE_BURST: updateBurst(); break;
        case MODE_CALIBRATION: updateCalibration(); break;
        case MODE_ERROR: updateError(); break;
    }
}

// ----------------------------------------------------
// MODE IMPLEMENTATIONS
// ----------------------------------------------------

void updateIdle() {
    pumpMotor.setTargetPWM(0);
    lcdDisplay.showBoot(); // Or Menu
    
    // Check Inputs to switch mode
    InputEvent e = userInput.getEvent();
    if (e == EVENT_MENU_MANUAL) currentMode = MODE_MANUAL;
    if (e == EVENT_START) currentMode = MODE_SMART;
    // if (e == EVENT_BURST) ... need to map burst key
    
    // Allow Tare -> Redirect to Calibration
    if (e == EVENT_TARE) {
        currentMode = MODE_CALIBRATION;
    }
    
    // Status Update
    lcdDisplay.showManual(pressureSensor.getValue(), 0, 0, voltageSensor.getValue(), currentSensor.getValue(), "IDLE");
}

void updateManual() {
    // Input Handling
    InputEvent e = userInput.getEvent();
    if (e == EVENT_STOP) {
        currentMode = MODE_IDLE;
        manualPWM = 0;
        pumpMotor.setTargetPWM(0);
        pid_integral = 0;
        pid_lastError = 0;
        return;
    }
    
    if (!manualTargetMode) {
        // PWM Control
        if (e == EVENT_UP && manualPWM < 255) manualPWM += 5;
        if (e == EVENT_DOWN && manualPWM > 0) manualPWM -= 5;
        pumpMotor.setTargetPWM(manualPWM);
    } else {
        // Target Pressure Control (PID)
        if (e == EVENT_UP) manualTargetPressure += 1.0;
        if (e == EVENT_DOWN) manualTargetPressure -= 1.0;
        
        float currentPressure = pressureSensor.getValue();
        float error = manualTargetPressure - currentPressure;
        
        unsigned long now = millis();
        if (pid_lastTime == 0) {
            pid_lastTime = now - 50; // default startup delta
            pid_lastError = error;
        }
        float dt = (now - pid_lastTime) / 1000.0f;
        if (dt <= 0.001f) dt = 0.001f;
        pid_lastTime = now;
        
        // Integrate only if error is reasonably small to prevent integral windup
        if (abs(error) < 10.0f) {
            pid_integral += error * dt;
        } else {
            pid_integral = 0; // Clear windup if far away
        }
        pid_integral = constrain(pid_integral, -100.0f, 100.0f); // anti-windup clamping
        
        float raw_derivative = (error - pid_lastError) / dt;
        static float derivative = 0;
        derivative = 0.7f * derivative + 0.3f * raw_derivative; // filter D-kick
        
        pid_lastError = error;
        
        float output = (pid_Kp * error) + (pid_Ki * pid_integral) + (pid_Kd * derivative);
        int targetPWM = constrain((int)output, 0, 255);
        
        pumpMotor.setTargetPWM(targetPWM);
    }
    
    lcdDisplay.showManual(pressureSensor.getValue(), 0, pumpMotor.getCurrentPWM(), 
                          voltageSensor.getValue(), currentSensor.getValue(), "MAN");
}

void updateSmart() {
    InputEvent e = userInput.getEvent();
    if (e == EVENT_STOP) {
        currentMode = MODE_IDLE;
        smartState = SMART_IDLE;
        pumpMotor.setTargetPWM(0);
        return;
    }
    
    float p = pressureSensor.getValue();
    
    // Always add points to history inside active inflation
    if (smartState == SMART_INFLATING_YIELD) {
        addHistoryPoint(p);
    }
    
    switch (smartState) {
        case SMART_IDLE:
            resetHistory();
            flatCount = 0; // explicit reset
            smartState = SMART_INFLATING_YIELD;
            startTime = millis();
            peakPressureTracker = 0;
            detectedYieldPressure = 0;
            pumpMotor.setTargetPWM(200); // Start
            break;
            
        case SMART_INFLATING_YIELD:
            if (p > peakPressureTracker) peakPressureTracker = p;
            
            // Smarter Yield detection using sliding window dP/dt
            {
                float slope = calculateSlope();
                
                // If we are past 5kPa and the inflation rate drops (yielding)
                if (p > 5.0f && slope <= 0.15f) { // Yield threshold of 0.15 kPa/s
                    flatCount++;
                    if (flatCount >= 10) { // ~100ms of sustained flattening
                        detectedYieldPressure = peakPressureTracker;
                        smartState = SMART_INFLATING_TARGET;
                        flatCount = 0;
                    }
                } else {
                    flatCount = 0;
                }
            }
            break;
            
        case SMART_INFLATING_TARGET:
            // Go to Ratio
            float target = detectedYieldPressure * userYieldRatio;
            if (p >= target) {
                pumpMotor.setTargetPWM(0);
                smartState = SMART_FINISHED;
            }
            break;
            
        case SMART_FINISHED:
            // Wait for user to stop
             pumpMotor.setTargetPWM(0);
            break;
    }
    
    lcdDisplay.showSmart(p, peakPressureTracker, detectedYieldPressure, 
                         detectedYieldPressure * userYieldRatio, 
                         (smartState == SMART_FINISHED) ? "DONE" : "RUN");
}

void updateBurst() {
    InputEvent e = userInput.getEvent();
    if (e == EVENT_STOP || e == EVENT_BACK || e == EVENT_SELECT) {
        currentMode = MODE_IDLE;
        pumpMotor.setTargetPWM(0);
        burstPopped = false;
        burstPeak = 0;
        return;
    }
    
    if (burstPopped) {
        // Hold pop result screen on LCD until user acknowledges
        if (pumpMotor.getCurrentPWM() > 0) pumpMotor.emergencyStop();
        lcdDisplay.showBurstPopped(burstPeak, currentSensor.getValue());
        return;
    }
    
    float p = pressureSensor.getValue();
    if (p > burstPeak) burstPeak = p;
    
    pumpMotor.setTargetPWM(255); // Full power
    
    // Pop Detection (Pressure drops 20% from Peak)
    if (burstPeak > 10.0f && p < burstPeak * 0.8f) {
        // POP! Freeze peak results on LCD
        pumpMotor.emergencyStop();
        burstPopped = true;
    }
    
    lcdDisplay.showBurst(p, burstPeak, currentSensor.getValue(), "RUN");
}

void updateCalibration() {
    static bool inAutoCal = false;
    static bool autoCalDone = false;
    static unsigned long lastAutoCal = 0;
    
    InputEvent e = userInput.getEvent();
    
    if (e == EVENT_BACK) {
        currentMode = MODE_IDLE;
        inAutoCal = false;
        autoCalDone = false;
        return;
    }

    if (!inAutoCal) {
        // Standard Menu
        lcdDisplay.showCalMenu(0, pressureSensor.getValue(), 0);
        
        // If SELECT (Rotary click or 5) and BMP is available, enter AutoCal
        if (e == EVENT_SELECT) {
            #ifdef USE_BMP280_REF
            inAutoCal = true;
            autoCalDone = false;
            lastAutoCal = millis();
            #else
            // Just behave like a regular click if no BMP support
            #endif
        }
    } else {
        // Auto Calibration Logic
        float refP = getRefPressure();
        float rawV = pressureSensor.getRawValue();
        
        if (!autoCalDone && millis() - lastAutoCal > 1000) {
            // Wait for pressure to stabilize and then Save point
            pressureSensor.addCalibrationPoint(rawV, refP);
            autoCalDone = true;
        }
        
        lcdDisplay.showAutoCal(pressureSensor.getValue(), refP, autoCalDone);
        
        if (e == EVENT_SELECT && autoCalDone) {
            saveSystemConfig(); // Persist the new point
            inAutoCal = false; // Go back
            autoCalDone = false;
        }
    }
}

void updateError() {
    lcdDisplay.showError(errorLine1, errorLine2);
    InputEvent e = userInput.getEvent();
    // Clear error on Stop/Select
    if (e == EVENT_STOP || e == EVENT_SELECT || e == EVENT_START) {
        currentMode = MODE_IDLE;
        pumpMotor.emergencyStop();
        pumpMotor.clearSafetyLockout();
    }
}
