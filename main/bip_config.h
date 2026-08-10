#ifndef BIP_CONFIG_H
#define BIP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

// ==========================================
// HARDWARE PIN DEFINITIONS (ESP32)
// ==========================================

// ADS1220 24-bit SPI ADC Pins
#define PIN_ADS_CS          GPIO_NUM_15
#define PIN_ADS_DRDY        GPIO_NUM_17
#define PIN_SPI_MOSI        GPIO_NUM_23
#define PIN_SPI_MISO        GPIO_NUM_19
#define PIN_SPI_SCLK        GPIO_NUM_18

// Analog Sensing Pins (ADC1 Channels)
#define PIN_VOLTAGE_ADC_CH  ADC_CHANNEL_6 // GPIO 34
#define PIN_CURRENT_ADC_CH  ADC_CHANNEL_7 // GPIO 35

// PWM Actuator Outputs
#define PIN_MOTOR_PWM       GPIO_NUM_3
#define PIN_SOLENOID_VALVE  GPIO_NUM_12

// ==========================================
// PWM & SENSOR CONFIGURATION
// ==========================================

#define MOTOR_PWM_FREQ      25000 // 25 kHz (Ultrasonic, silent drive)
#define MOTOR_PWM_CHANNEL   LEDC_CHANNEL_0
#define VALVE_PWM_FREQ      20000 // 20 kHz
#define VALVE_PWM_CHANNEL   LEDC_CHANNEL_1
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_RESOLUTION      LEDC_TIMER_8_BIT // 0-255

// WiFi & Network Configuration
#define WIFI_SSID           "BIP_AP"
#define WIFI_PASSWORD       "12345678"
#define UDP_PORT            8888
#define UDP_BROADCAST_IP    "255.255.255.255"

// Safety Default Limits
#define SAFETY_UV_DEFAULT   10.5f // Volts
#define SAFETY_OC_DEFAULT   4.5f  // Amps
#define SAFETY_SAG_DEFAULT  10    // % Drop allowed

// Telemetry Buffer & FreeRTOS Settings
#define TELEM_BUFFER_SIZE   10
#define TELEM_RING_BUF_SIZE (32 * 1024) // 32 KB Lock-Free RingBuffer

// Full-Precision Pop Black Box RAM Buffer (2000 Samples @ 2000 SPS = 1.0 Sec Full Precision)
#define POP_BUF_SIZE        2000

// Task Priorities & Cores
#define CONTROL_TASK_PRIO   12 // Real-time priority
#define CONTROL_TASK_CORE   1  // Dedicated Core 1 for DSP/Control
#define NET_TASK_PRIO       5
#define NET_TASK_CORE       0  // Dedicated Core 0 for WiFi/BSD Sockets

// NVS Storage Namespace
#define NVS_NAMESPACE       "bip_storage"
#define NVS_CFG_KEY         "sys_cfg"

// Telemetry Packet Structure
typedef struct {
    double timestamp;
    float pressure;
    int pwm;
    float voltage;
    float current;
    uint8_t mode;
    float rawPressure;
    float rawVoltage;
    float rawCurrent;
    float mcuTemp;
} TelemetrySample;

// Full Precision Pop Black Box Sample Structure (16 bytes)
typedef struct {
    uint32_t timestamp_us; // Microsecond timestamp
    float pressure;        // Calibrated pressure (kPa)
    float rawVolts;        // Raw 24-bit ADC voltage
    uint16_t pwm;          // Motor PWM
    uint16_t current_ma;   // Motor current (mA)
} PopBlackBoxSample;

#endif // BIP_CONFIG_H
