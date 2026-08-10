#include "Sensors.h"

#ifdef USE_BMP280_REF
Adafruit_BMP280 bmp;
bool bmpAvailable = false;
#endif

// Global Instances
Sensor pressureSensor(PIN_PRESSURE_SENSOR, "Pressure");
Sensor voltageSensor(PIN_VOLTAGE_SENSOR, "Voltage");
Sensor currentSensor(PIN_CURRENT_SENSOR, "Current");

void initSensors() {
    pressureSensor.begin();
    voltageSensor.begin();
    currentSensor.begin();
    
    #ifdef USE_BMP280_REF
    // Try standard addresses
    if (bmp.begin(0x76)) {
        bmpAvailable = true; 
    } else if (bmp.begin(0x77)) {
        bmpAvailable = true;
    }
    #endif
    
    // Check if Calibration is Empty (First Boot), set specific defaults
    if (pressureSensor.getCalibration().numPoints == 0) {
        // Default Linear Map for Selected Sensor
        // 0.5V -> 0kPa, 4.5V -> PRESS_MAX_KPA
        pressureSensor.addCalibrationPoint(PRESS_MIN_V, 0.0);
        pressureSensor.addCalibrationPoint(PRESS_MAX_V, PRESS_MAX_KPA);
    }
    
    if (voltageSensor.getCalibration().numPoints == 0) {
        // Default Input Voltage vs Real Voltage
        voltageSensor.addCalibrationPoint(0.0, 0.0);
        voltageSensor.addCalibrationPoint(5.0, 5.0 * DEFAULT_VOLT_FACTOR); 
    }
    
    if (currentSensor.getCalibration().numPoints == 0) {
        // ACS712 Default
        // 2.5V (Zero Clean) -> 0A
        // 2.5V + Sensitivity -> 1A
        currentSensor.addCalibrationPoint(ACS712_ZERO_V, 0.0);
        currentSensor.addCalibrationPoint(ACS712_ZERO_V + ACS712_SENSITIVITY, 1.0);
    }
}

float getRefPressure() {
    #ifdef USE_BMP280_REF
    if (bmpAvailable) return bmp.readPressure() / 1000.0f; // Pa -> kPa
    #endif
    return 0.0f;
}

void updateSensors() {
    pressureSensor.read();
    voltageSensor.read();
    currentSensor.read();
}

void tareAllSensors() {
    pressureSensor.tare();
}

// ==========================================
// Sensor Class Implementation
// ==========================================

Sensor::Sensor(uint8_t pin, String name) {
    _pin = pin;
    _name = name;
    _windowSize = 10; // Default smoothing
    _histIndex = 0;
    _runningSum = 0;
    _zeroOffset = 0.0f;
    _calData.numPoints = 0;
    _currentValue = 0.0f;
    _rawVolts = 0.0f;
}

void Sensor::begin() {
    pinMode(_pin, INPUT);
    // Initialize filter history
    for(int i=0; i<20; i++) _history[i] = 0;
}

void Sensor::setMovingAverageWindow(uint8_t size) {
    if (size > 20) size = 20;
    if (size < 1) size = 1;
    _windowSize = size;
}

void Sensor::read() {
    int rawADC = analogRead(_pin);
    float volts = rawADC * (V_REF / ADC_MAX_VAL);
    setRawValue(volts);
}

void Sensor::setRawValue(float volts) {
    // Moving Average Filter
    _runningSum -= _history[_histIndex];
    _history[_histIndex] = volts;
    _runningSum += _history[_histIndex];
    
    _histIndex++;
    if (_histIndex >= _windowSize) {
        _histIndex = 0;
        // Periodic recalculation to fix floating point drift
        float trueSum = 0;
        for (int i=0; i<_windowSize; i++) trueSum += _history[i];
        _runningSum = trueSum;
    }
    
    _rawVolts = _runningSum / _windowSize;
    _currentValue = mapMultiPoint(_rawVolts);
}

float Sensor::getRawValue() {
    return _rawVolts;
}

float Sensor::getValue() {
    return _currentValue - _zeroOffset;
}

void Sensor::tare() {
    _zeroOffset = _currentValue;
}

void Sensor::resetTare() {
    _zeroOffset = 0.0f;
}

float Sensor::getZeroOffset() {
    return _zeroOffset;
}

bool Sensor::isFault() {
    // Basic wiring check: detect open or short circuit pin reads (<0.02V or >4.98V)
    if (_rawVolts < 0.02f || _rawVolts > 4.98f) return true;
    return false;
}

String Sensor::getFaultType() {
    if (_rawVolts < 0.02f) return _name + "_SHORT_GND";
    if (_rawVolts > 4.98f) return _name + "_SHORT_VCC";
    return "OK";
}

// --- Calibration Logic ---

void Sensor::resetCalibration() {
    _calData.numPoints = 0;
    _zeroOffset = 0.0f;
}

void Sensor::addCalibrationPoint(float measuredRaw, float knownReal) {
    if (_calData.numPoints >= MAX_CAL_POINTS) return;
    
    // Prevent duplicate raw values to avoid division by zero
    for (int i = 0; i < _calData.numPoints; i++) {
        if (abs(_calData.points[i].rawValue - measuredRaw) < 1e-6f) return;
    }
    
    _calData.points[_calData.numPoints].rawValue = measuredRaw;
    _calData.points[_calData.numPoints].realValue = knownReal;
    _calData.numPoints++;
    
    // Bubble Sort by Raw Value to keep points ordered
    for (int i = 0; i < _calData.numPoints - 1; i++) {
        for (int j = 0; j < _calData.numPoints - i - 1; j++) {
            if (_calData.points[j].rawValue > _calData.points[j+1].rawValue) {
                CalibrationPoint temp = _calData.points[j];
                _calData.points[j] = _calData.points[j+1];
                _calData.points[j+1] = temp;
            }
        }
    }
}

SensorCalibration Sensor::getCalibration() {
    return _calData;
}

void Sensor::setCalibration(SensorCalibration cal) {
    _calData = cal;
}

float Sensor::mapMultiPoint(float x) {
    if (_calData.numPoints == 0) return x; // No Cal? Return raw
    if (_calData.numPoints == 1) return x; 
    
    // Extrapolation Low
    if (x <= _calData.points[0].rawValue) {
        float denom = _calData.points[1].rawValue - _calData.points[0].rawValue;
        if (abs(denom) < 1e-9f) return _calData.points[0].realValue;
        float slope = (_calData.points[1].realValue - _calData.points[0].realValue) / denom;
        return _calData.points[0].realValue - ((_calData.points[0].rawValue - x) * slope);
    }
    
    // Interpolation
    for (int i = 0; i < _calData.numPoints - 1; i++) {
        if (x >= _calData.points[i].rawValue && x <= _calData.points[i+1].rawValue) {
             float denom = _calData.points[i+1].rawValue - _calData.points[i].rawValue;
             if (abs(denom) < 1e-9f) return _calData.points[i].realValue;
             float slope = (_calData.points[i+1].realValue - _calData.points[i].realValue) / denom;
             return _calData.points[i].realValue + ((x - _calData.points[i].rawValue) * slope);
        }
    }
    
    // Extrapolation High
    int last = _calData.numPoints - 1;
    float denom = _calData.points[last].rawValue - _calData.points[last-1].rawValue;
    if (abs(denom) < 1e-9f) return _calData.points[last].realValue;
    float slope = (_calData.points[last].realValue - _calData.points[last-1].realValue) / denom;
    return _calData.points[last].realValue + ((x - _calData.points[last].rawValue) * slope);
}
