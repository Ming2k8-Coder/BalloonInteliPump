#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// HARDWARE CONFIGURATION
// ==========================================

// --- Feature Flags ---
// Comment this out to use 4x4 Keypad instead of Rotary Encoder
// #define USE_ROTARY_ENCODER 

// --- Pin Definitions ---
// Analog Inputs
#define PIN_PRESSURE_SENSOR A0
#define PIN_CURRENT_SENSOR  A1 // ACS712
#define PIN_VOLTAGE_SENSOR  A2 // Voltage Divider

// I2C Pins (LCD) - Defined by Hardware (A4/A5 on Uno/Nano)
// #define PIN_SDA A4
// #define PIN_SCL A5

// Digital Outputs
#define PIN_MOTOR_PWM       3  // IRLZ44N Gate
#define PIN_SOLENOID_VALVE  12 // Solenoid Relief Valve (12V MOSFET)

// Inputs (Keypad or Rotary)
#ifdef USE_ROTARY_ENCODER
    #define PIN_ENC_CLK     2  // Interrupt capable
    #define PIN_ENC_DT      4
    #define PIN_ENC_SW      5
#else
    // 4x4 Matrix Keypad
    // Row Pins (Outputs)
    #define PIN_KEY_R1      4
    #define PIN_KEY_R2      5
    #define PIN_KEY_R3      6
    #define PIN_KEY_R4      7
    // Column Pins (Inputs)
    #define PIN_KEY_C1      8
    #define PIN_KEY_C2      9
    #define PIN_KEY_C3      10
    #define PIN_KEY_C4      11
#endif

// ==========================================
// SYSTEM SETTINGS
// ==========================================

#define SERIAL_BAUD_RATE    115200

// --- ESP32 High-Frequency PWM Settings ---
#define MOTOR_PWM_FREQ      25000 // 25 kHz (Ultrasonic, silent motor drive)
#define MOTOR_PWM_CHANNEL   0     // LEDC Channel 0
#define VALVE_PWM_FREQ      20000 // 20 kHz
#define VALVE_PWM_CHANNEL   1     // LEDC Channel 1
#define PWM_RESOLUTION      8     // 8-bit resolution (0-255)

// --- Sensor Settings ---
#define ADC_MAX_VAL         1023.0f
#define V_REF               5.0f

// --- Sensor Type Configuration ---
#define SENSOR_TYPE_XGZP6847A   0 // 0-200kPa
#define SENSOR_TYPE_MPS20N0040D 1 // 0-40kPa

// SELECT SENSOR HERE:
#define SENSOR_TYPE SENSOR_TYPE_XGZP6847A
// #define SENSOR_TYPE SENSOR_TYPE_MPS20N0040D 

// BMP280/BME280 Reference for Auto-Calibration
// #define USE_BMP280_REF 

// Pressure Sensor Settings
#define PRESS_MIN_V         0.5f
#define PRESS_MAX_V         4.5f

#if SENSOR_TYPE == SENSOR_TYPE_MPS20N0040D
    #define PRESS_MAX_KPA   40.0f
#else
    #define PRESS_MAX_KPA   200.0f // Default XGZP6847A
#endif

// Current Sensor (ACS712)
// ACS712-05B: 185mV/A
// ACS712-20A: 100mV/A
// ACS712-30A: 66mV/A
#define ACS712_SENSITIVITY  0.185f // Default 5A module (Adjust in Cal)
#define ACS712_ZERO_V       2.5f

// Voltage Divider
// R1 = 10k, R2 = 2.2k -> Divider = 2.2 / (10 + 2.2) = 0.1803
// V_in = V_out / 0.1803
// Max V_in for 5V out = 5 / 0.1803 = 27.7V
#define VOLT_DIV_R1         10000.0f
#define VOLT_DIV_R2         2200.0f
#define DEFAULT_VOLT_FACTOR ((VOLT_DIV_R1 + VOLT_DIV_R2) / VOLT_DIV_R2)

// --- EEPROM Addresses ---
// Wear Leveling: We divide EEPROM into slots.
// Slot Structure: [Magic(2)][Pressure(Cal)][Voltage(Cal)][Current(Cal)]
// Total Size per Slot approx 2 + 128 = 130 bytes.
// Arduino Nano EEPROM = 1024 bytes.
// 1024 / 150 = ~6 slots.
#define EEPROM_ROW_SIZE     150 
#define EEPROM_SLOTS        6
#define EEPROM_MAGIC_VAL    0x4247 // Combined magic

// --- Safety Limits (Defaults) ---
#define SAFETY_UV_DEFAULT   10.5f // Volts
#define SAFETY_OC_DEFAULT   4.5f  // Amps
#define SAFETY_SAG_DEFAULT  10    // % Drop allowed

// --- WiFi Settings (ESP32 only) ---
#define WIFI_SSID           "BIP_AP"       // Access Point SSID or local station SSID
#define WIFI_PASSWORD       "12345678"     // WiFi Password (min 8 chars)
#define UDP_PORT            8888
#define UDP_BROADCAST_IP    "255.255.255.255"

// --- ADS1220 SPI ADC Settings (ESP32 only) ---
#define PIN_ADS_CS          15 // Changed from 5 to 15 to avoid VSPI CS/JTAG conflict on some ESP32 dev boards
#define PIN_ADS_DRDY        17 // DRDY interrupt from ADS1220. Note: GPIO17 may conflict with UART2 TX on some boards. Verify your ESP32 pinout.

// --- Control Loop ---
#define LOOP_INTERVAL_MS    0    // MAX Control Loop
#define WDT_TIMEOUT_MS      2000  // Watchdog Timeout

#endif // CONFIG_H
