#ifndef BIP_SENSORS_H
#define BIP_SENSORS_H

#include "bip_config.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"

#define MAX_CAL_POINTS 5

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
};

// Global Sensor Objects
extern BIP_Sensor pressureSensor;
extern BIP_Sensor voltageSensor;
extern BIP_Sensor currentSensor;

// Driver Functions
esp_err_t init_spi_ads1220(spi_device_handle_t *spi_handle);
int32_t read_ads1220_raw(spi_device_handle_t spi_handle);
esp_err_t init_internal_adc(adc_oneshot_unit_handle_t *adc_handle);
void update_adc_sensors(adc_oneshot_unit_handle_t adc_handle);

#endif // BIP_SENSORS_H
