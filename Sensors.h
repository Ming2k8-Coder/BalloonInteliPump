#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Config.h"

#ifdef USE_BMP280_REF
#include <Wire.h>
#include <Adafruit_BMP280.h>
#endif

// Max calibration points for multipoint mapping
#define MAX_CAL_POINTS 5

struct CalibrationPoint {
    float rawValue;  // ADC Reading or Voltage
    float realValue; // Physical Unit (kPa, V, A)
};

struct SensorCalibration {
    uint8_t numPoints;
    CalibrationPoint points[MAX_CAL_POINTS];
};

class Sensor {
public:
    Sensor(uint8_t pin, String name);
    void begin();
    void read();
    void setRawValue(float rawVolts); // Direct injector for ADS1220 external ADC
    float getValue(); // Returns the calibrated value minus zero offset
    float getRawValue(); // Returns raw ADC/Voltage
    
    // Tare / Zero Offset
    void tare();
    void resetTare();
    float getZeroOffset();
    
    // Fault Diagnostics
    bool isFault();
    String getFaultType();
    
    // Calibration Methods
    void resetCalibration();
    void addCalibrationPoint(float measuredRaw, float knownReal);
    void setCalibration(SensorCalibration cal);
    SensorCalibration getCalibration();
    
    // Utilities
    void setMovingAverageWindow(uint8_t size); // 1-20
    
protected:
    uint8_t _pin;
    String _name;
    float _currentValue;
    float _rawVolts;
    float _zeroOffset;
    
    // Smoothing
    float _history[20];
    uint8_t _histIndex;
    uint8_t _windowSize;
    float _runningSum;
    
    // Calibration Data
    SensorCalibration _calData;
    
    float mapMultiPoint(float val);
};

// Specialized Managers
extern Sensor pressureSensor;
extern Sensor voltageSensor;
extern Sensor currentSensor;

#ifdef USE_BMP280_REF
extern Adafruit_BMP280 bmp;
#endif

void initSensors();
void updateSensors();
void tareAllSensors();
float getRefPressure(); // Returns kPa or 0 if optional sensor missing

#endif // SENSORS_H
