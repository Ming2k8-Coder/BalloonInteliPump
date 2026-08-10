#ifndef BIP_SENSORS_H
#define BIP_SENSORS_H

#include "bip_config.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MAX_CAL_POINTS 5

// 1D Adaptive Kalman Filter for Zero-Lag Pressure Signal Filtering
class AdaptiveKalmanFilter {
public:
    AdaptiveKalmanFilter(float q = 0.02f, float r = 0.5f, float p = 1.0f) {
        _q = q; // Process noise covariance
        _r = r; // Measurement noise covariance
        _p = p; // Estimation error covariance
        _x = 0.0f;
        _k = 0.0f;
    }

    float update(float measurement) {
        _p = _p + _q;
        _k = _p / (_p + _r);
        _x = _x + _k * (measurement - _x);
        _p = (1.0f - _k) * _p;
        return _x;
    }

    void reset(float initial_val = 0.0f) {
        _x = initial_val;
        _p = 1.0f;
    }

    float getValue() const { return _x; }

private:
    float _q, _r, _p, _x, _k;
};

typedef struct {
    float rawValue;  // ADC Reading or Voltage
    float realValue; // Physical Unit (kPa, V, A)
} CalibrationPoint;

typedef struct {
    uint8_t numPoints;
    CalibrationPoint points[MAX_CAL_POINTS];
} SensorCalibration;

class BIP_Sensor {
public:
    BIP_Sensor(const char* name);
    void setRawValue(float rawVolts);
    float getValue();
    float getRawValue();
    float getFilteredValue(); // Kalman filtered value
    
    // Tare zeroing
    void tare();
    void resetTare();
    float getZeroOffset();
    
    // Calibration
    void resetCalibration();
    void addCalibrationPoint(float measuredRaw, float knownReal);
    void setCalibration(SensorCalibration cal);
    SensorCalibration getCalibration();
    
    float mapMultiPoint(float x);

private:
    const char* _name;
    float _currentValue;
    float _rawVolts;
    float _zeroOffset;
    
    float _history[20];
    uint8_t _histIndex;
    uint8_t _windowSize;
    float _runningSum;
    
    SensorCalibration _calData;
    AdaptiveKalmanFilter _kalman;
};

// Global Sensor Objects
extern BIP_Sensor pressureSensor;
extern BIP_Sensor voltageSensor;
extern BIP_Sensor currentSensor;

// Driver Functions
esp_err_t init_spi_ads1220(spi_device_handle_t *spi_handle);
int32_t read_ads1220_raw(spi_device_handle_t spi_handle);
esp_err_t init_drdy_isr(TaskHandle_t targetTaskHandle);
esp_err_t init_internal_adc(adc_oneshot_unit_handle_t *adc_handle);
void update_adc_sensors(adc_oneshot_unit_handle_t adc_handle);
esp_err_t init_mcu_temp_sensor();
float read_mcu_temp();

#endif // BIP_SENSORS_H
