#include "bip_sensors.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include <cmath>
#include <cstring>

static const char *TAG = "BIP_SENSORS";

BIP_Sensor pressureSensor("Pressure");
BIP_Sensor voltageSensor("Voltage");
BIP_Sensor currentSensor("Current");

static TaskHandle_t s_controlTaskHandle = NULL;
static temperature_sensor_handle_t temp_handle = NULL;

static void IRAM_ATTR drdy_gpio_isr_handler(void* arg) {
    if (s_controlTaskHandle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(s_controlTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

BIP_Sensor::BIP_Sensor(const char* name) : _kalman(0.02f, 0.5f, 1.0f) {
    _name = name;
    _windowSize = 10;
    _histIndex = 0;
    _runningSum = 0;
    _zeroOffset = 0.0f;
    _currentValue = 0.0f;
    _rawVolts = 0.0f;
    _calData.numPoints = 0;
    memset(_history, 0, sizeof(_history));
}

void BIP_Sensor::setRawValue(float volts) {
    _runningSum -= _history[_histIndex];
    _history[_histIndex] = volts;
    _runningSum += _history[_histIndex];
    
    _histIndex++;
    if (_histIndex >= _windowSize) {
        _histIndex = 0;
        float trueSum = 0;
        for (int i = 0; i < _windowSize; i++) trueSum += _history[i];
        _runningSum = trueSum;
    }
    
    _rawVolts = _runningSum / _windowSize;
    float unFiltered = mapMultiPoint(_rawVolts);
    _currentValue = _kalman.update(unFiltered);
}

float BIP_Sensor::getRawValue() {
    return _rawVolts;
}

float BIP_Sensor::getValue() {
    return _currentValue - _zeroOffset;
}

float BIP_Sensor::getFilteredValue() {
    return _kalman.getValue() - _zeroOffset;
}

void BIP_Sensor::tare() {
    _zeroOffset = _currentValue;
}

void BIP_Sensor::resetTare() {
    _zeroOffset = 0.0f;
}

float BIP_Sensor::getZeroOffset() {
    return _zeroOffset;
}

void BIP_Sensor::resetCalibration() {
    _calData.numPoints = 0;
    _zeroOffset = 0.0f;
    _kalman.reset(0.0f);
}

void BIP_Sensor::addCalibrationPoint(float measuredRaw, float knownReal) {
    if (_calData.numPoints >= MAX_CAL_POINTS) return;
    
    for (int i = 0; i < _calData.numPoints; i++) {
        if (std::abs(_calData.points[i].rawValue - measuredRaw) < 1e-6f) return;
    }
    
    _calData.points[_calData.numPoints].rawValue = measuredRaw;
    _calData.points[_calData.numPoints].realValue = knownReal;
    _calData.numPoints++;
    
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

SensorCalibration BIP_Sensor::getCalibration() {
    return _calData;
}

void BIP_Sensor::setCalibration(SensorCalibration cal) {
    _calData = cal;
}

float BIP_Sensor::mapMultiPoint(float x) {
    if (_calData.numPoints < 2) return x;
    
    if (x <= _calData.points[0].rawValue) {
        float denom = _calData.points[1].rawValue - _calData.points[0].rawValue;
        if (std::abs(denom) < 1e-9f) return _calData.points[0].realValue;
        float slope = (_calData.points[1].realValue - _calData.points[0].realValue) / denom;
        return _calData.points[0].realValue - ((_calData.points[0].rawValue - x) * slope);
    }
    
    for (int i = 0; i < _calData.numPoints - 1; i++) {
        if (x >= _calData.points[i].rawValue && x <= _calData.points[i+1].rawValue) {
             float denom = _calData.points[i+1].rawValue - _calData.points[i].rawValue;
             if (std::abs(denom) < 1e-9f) return _calData.points[i].realValue;
             float slope = (_calData.points[i+1].realValue - _calData.points[i].realValue) / denom;
             return _calData.points[i].realValue + ((x - _calData.points[i].rawValue) * slope);
        }
    }
    
    int last = _calData.numPoints - 1;
    float denom = _calData.points[last].rawValue - _calData.points[last-1].rawValue;
    if (std::abs(denom) < 1e-9f) return _calData.points[last].realValue;
    float slope = (_calData.points[last].realValue - _calData.points[last-1].realValue) / denom;
    return _calData.points[last].realValue + ((x - _calData.points[last].rawValue) * slope);
}

// ADS1220 SPI Driver Initialization
esp_err_t init_spi_ads1220(spi_device_handle_t *spi_handle) {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_SPI_MOSI;
    buscfg.miso_io_num = PIN_SPI_MISO;
    buscfg.sclk_io_num = PIN_SPI_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 32;

    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 4 * 1000 * 1000;
    devcfg.mode = 1;
    devcfg.spics_io_num = PIN_ADS_CS;
    devcfg.queue_size = 7;

    ret = spi_bus_add_device(HSPI_HOST, &devcfg, spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ADS1220 device to SPI bus");
        return ret;
    }

    uint8_t tx_data[5] = { 0x43, 0x81, 0xCE, 0x00, 0x00 };
    spi_transaction_t t = {};
    t.length = 8 * 5;
    t.tx_buffer = tx_data;
    spi_device_polling_transmit(*spi_handle, &t);

    uint8_t start_cmd = 0x08;
    t.length = 8;
    t.tx_buffer = &start_cmd;
    spi_device_polling_transmit(*spi_handle, &t);

    ESP_LOGI(TAG, "ADS1220 SPI driver initialized at 2000 SPS with Polling Transmit");
    return ESP_OK;
}

int32_t read_ads1220_raw(spi_device_handle_t spi_handle) {
    uint8_t tx_cmd = 0x12;
    uint8_t rx_data[3] = {0};

    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &tx_cmd;
    spi_device_polling_transmit(spi_handle, &t);

    t.length = 8 * 3;
    t.tx_buffer = NULL;
    t.rx_buffer = rx_data;
    spi_device_polling_transmit(spi_handle, &t);

    int32_t raw_val = ((int32_t)rx_data[0] << 16) | ((int32_t)rx_data[1] << 8) | rx_data[2];
    if (raw_val & 0x800000) {
        raw_val |= 0xFF000000;
    }
    return raw_val;
}

esp_err_t init_drdy_isr(TaskHandle_t targetTaskHandle) {
    s_controlTaskHandle = targetTaskHandle;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << PIN_ADS_DRDY);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ADS_DRDY, drdy_gpio_isr_handler, NULL);

    ESP_LOGI(TAG, "Hardware DRDY GPIO interrupt installed with Direct Task Notification");
    return ESP_OK;
}

esp_err_t init_internal_adc(adc_oneshot_unit_handle_t *adc_handle) {
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;

    esp_err_t ret = adc_oneshot_new_unit(&init_config, adc_handle);
    if (ret != ESP_OK) return ret;

    adc_oneshot_chan_cfg_t config = {};
    config.bitwidth = ADC_BITWIDTH_DEFAULT;
    config.atten = ADC_ATTEN_DB_12;

    adc_oneshot_config_channel(*adc_handle, PIN_VOLTAGE_ADC_CH, &config);
    adc_oneshot_config_channel(*adc_handle, PIN_CURRENT_ADC_CH, &config);

    return ESP_OK;
}

void update_adc_sensors(adc_oneshot_unit_handle_t adc_handle) {
    int rawV = 0, rawI = 0;
    adc_oneshot_read(adc_handle, PIN_VOLTAGE_ADC_CH, &rawV);
    adc_oneshot_read(adc_handle, PIN_CURRENT_ADC_CH, &rawI);

    float volts_v = (rawV / 4095.0f) * 3.3f * 4.0f;
    float amps_i = ((rawI / 4095.0f) * 3.3f - 1.65f) / 0.185f;

    voltageSensor.setRawValue(volts_v);
    currentSensor.setRawValue(amps_i < 0.0f ? 0.0f : amps_i);
}

esp_err_t init_mcu_temp_sensor() {
    temperature_sensor_config_t temp_sensor_config = {
        .range_min = 10,
        .range_max = 80,
    };
    esp_err_t ret = temperature_sensor_install(&temp_sensor_config, &temp_handle);
    if (ret == ESP_OK) {
        temperature_sensor_enable(temp_handle);
        ESP_LOGI(TAG, "MCU Internal Junction Temperature Sensor initialized");
    }
    return ret;
}

float read_mcu_temp() {
    float tsens_value = 0.0f;
    if (temp_handle != NULL) {
        temperature_sensor_get_celsius(temp_handle, &tsens_value);
    }
    return tsens_value;
}
