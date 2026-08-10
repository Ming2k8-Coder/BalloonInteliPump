#include "Config.h"
#include "Sensors.h"
#include "Motor.h"
#include "Input.h"
#include "Display.h"
#include "StateFunctions.h"
#include "Storage.h"

#ifdef ESP32
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
WiFiUDP udpClient;
bool wifiConnected = false;

// Volatile variable for raw 24-bit pressure from ADS1220
volatile int32_t adsRawPressure = 0;
volatile uint32_t totalSamplesProcessed = 0;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// High-speed Double Buffering Telemetry Structure
#define TELEM_BUFFER_SIZE 10

struct TelemetrySample {
    double timestamp;
    float pressure;
    int pwm;
    float voltage;
    float current;
    uint8_t mode;
    float rawPressure;
    float rawVoltage;
    float rawCurrent;
};

TelemetrySample bufferA[TELEM_BUFFER_SIZE];
TelemetrySample bufferB[TELEM_BUFFER_SIZE];

volatile int writeIndex = 0;
volatile bool useBufferA = true;
volatile bool bufferReadyToSend = false;
volatile bool sendingBufferA = false; // Flag to tell loop thread which buffer to send

volatile bool drdyTriggered = false;

void IRAM_ATTR ads1220_isr() {
    if (!isConnected || !wifiConnected) return;
    drdyTriggered = true;
}
#endif

unsigned long lastTelemetry = 0;
bool isConnected = false;

// Cross-version helper for Valve PWM
static void writeValvePWM(uint8_t val) {
#ifdef ESP32
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcWrite(PIN_SOLENOID_VALVE, val);
    #else
        ledcWrite(VALVE_PWM_CHANNEL, val);
    #endif
#else
    analogWrite(PIN_SOLENOID_VALVE, val);
#endif
}

void setup() {
#ifndef ESP32
    wdt_disable(); // Safe start
#endif
    
    Serial.begin(SERIAL_BAUD_RATE);
    
    // Init Modules
    initSensors(); 
    initMotor();   
    initStorage(); // Loads Configuration (Cal + Safety)
    
    // Configure Solenoid Relief Valve
#ifdef ESP32
    #if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcAttach(PIN_SOLENOID_VALVE, VALVE_PWM_FREQ, PWM_RESOLUTION);
    #else
        ledcSetup(VALVE_PWM_CHANNEL, VALVE_PWM_FREQ, PWM_RESOLUTION);
        ledcAttachPin(PIN_SOLENOID_VALVE, VALVE_PWM_CHANNEL);
    #endif
    writeValvePWM(0);
#else
    pinMode(PIN_SOLENOID_VALVE, OUTPUT);
    digitalWrite(PIN_SOLENOID_VALVE, LOW); // Closed by default
#endif

    userInput.begin();
    lcdDisplay.begin();
    initStates();

    #ifdef ESP32
    // Setup WiFi - try station mode first, fallback to AP
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long startAttempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 4000) {
        delay(100);
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
    } else {
        WiFi.disconnect();
        WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
        wifiConnected = true;
    }
    udpClient.begin(UDP_PORT);
    
    // Initialize SPI for ADS1220
    SPI.begin();
    pinMode(PIN_ADS_CS, OUTPUT);
    digitalWrite(PIN_ADS_CS, HIGH);
    pinMode(PIN_ADS_DRDY, INPUT_PULLUP);
    
    // Configure ADS1220 Registers
    digitalWrite(PIN_ADS_CS, LOW);
    SPI.transfer(0x43); // Write 4 registers starting at 0
    SPI.transfer(0x81); // Reg 0: Single-ended AIN0-AVSS, Gain 1, PGA bypassed
    SPI.transfer(0xCE); // Reg 1: 2000 SPS (Turbo Mode), Continuous Conversion, AVDD/AVSS VREF
    SPI.transfer(0x00); // Reg 2: IDAC off
    SPI.transfer(0x00); // Reg 3: Use internal clock (default)
    digitalWrite(PIN_ADS_CS, HIGH);
    
    // Start continuous conversion
    digitalWrite(PIN_ADS_CS, LOW);
    SPI.transfer(0x08); // START/SYNC command
    digitalWrite(PIN_ADS_CS, HIGH);
    
    // Attach ISR on DRDY falling edge (data ready)
    attachInterrupt(digitalPinToInterrupt(PIN_ADS_DRDY), ads1220_isr, FALLING);
    #endif
    
#ifndef ESP32
    wdt_enable(WDTO_2S);
#endif
}

void executeCommand(String cmd) {
    if (cmd == "CONNECT") {
        isConnected = true;
        bufferReadyToSend = false; // clear on connect
        Serial.println("CONNECTED");
    } else if (cmd == "START_MANUAL") {
        currentMode = MODE_MANUAL;
    } else if (cmd == "START_SMART") {
        currentMode = MODE_SMART;
    } else if (cmd == "START_BURST") {
        currentMode = MODE_BURST;
    } else if (cmd == "STOP") {
        pumpMotor.emergencyStop();
        currentMode = MODE_IDLE;
    } else if (cmd == "VALVE_ON") {
        writeValvePWM(255); // Open relief valve fully
        Serial.println("VALVE_OPEN");
    } else if (cmd == "VALVE_OFF") {
        writeValvePWM(0);   // Close relief valve fully
        Serial.println("VALVE_CLOSED");
    } else if (cmd.startsWith("SET_VALVE_PWM ") && cmd.length() > 14) {
        int val = cmd.substring(14).toInt();
        writeValvePWM(constrain(val, 0, 255));
        Serial.print("VALVE_PWM_SET ");
        Serial.println(val);
    } else if (cmd.startsWith("SET_PWM ")) {
        int val = cmd.substring(8).toInt();
        setManualMode(false); // Switch back to direct PWM
        pumpMotor.setTargetPWM(constrain(val, 0, 255));
    } else if (cmd.startsWith("SET_TARGET ")) {
        float val = cmd.substring(11).toFloat();
        setManualTarget(val);
    } else if (cmd.startsWith("SET_RAMP ")) {
        float rate = cmd.substring(9).toFloat();
        pumpMotor.setRampRate(rate);
        Serial.print("RAMP_SET ");
        Serial.println(rate);
    } else if (cmd.startsWith("SET_MAN_MODE ")) {
        bool mode = (cmd.substring(13).toInt() != 0);
        setManualMode(mode);
    } else if (cmd.startsWith("SET_RATIO ")) {
        float val = cmd.substring(10).toFloat();
        setYieldRatio(val);
    } else if (cmd.startsWith("SET_PID ")) {
        // Format: SET_PID <Kp> <Ki> <Kd>
        int firstSpace = cmd.indexOf(' ', 8);
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        if (firstSpace > 0 && secondSpace > 0) {
            float kp = cmd.substring(8, firstSpace).toFloat();
            float ki = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            float kd = cmd.substring(secondSpace + 1).toFloat();
            setPID(kp, ki, kd);
            Serial.println("PID_UPDATED");
        }
    } else if (cmd == "ZERO_PRESSURE" || cmd == "ZERO_TARE") {
        pressureSensor.tare();
        Serial.println("PRESSURE_ZEROED");
    } else if (cmd == "GET_DIAGNOSTICS" || cmd == "STATUS") {
        uint32_t freeHeap = 0;
        int rssi = 0;
#ifdef ESP32
        freeHeap = ESP.getFreeHeap();
        if (wifiConnected) rssi = WiFi.RSSI();
#endif
        Serial.print("DIAG,uptime=");
        Serial.print(millis() / 1000);
        Serial.print(",heap=");
        Serial.print(freeHeap);
        Serial.print(",rssi=");
        Serial.print(rssi);
        Serial.print(",mode=");
        Serial.print(currentMode);
        Serial.print(",zero_offset=");
        Serial.print(pressureSensor.getZeroOffset(), 3);
#ifdef ESP32
        Serial.print(",samples=");
        Serial.print(totalSamplesProcessed);
#endif
        Serial.println();
    } else if (cmd == "TARE" || cmd == "CAL_MODE") {
        pressureSensor.tare();
        currentMode = MODE_CALIBRATION;
        Serial.println("MODE_CALIBRATION");
    } else if (cmd.startsWith("CAL_CLR ")) {
        String sensor = cmd.substring(8);
        if (sensor == "pressure") pressureSensor.resetCalibration();
        else if (sensor == "voltage") voltageSensor.resetCalibration();
        else if (sensor == "current") currentSensor.resetCalibration();
        Serial.println("CAL_RESET_" + sensor);
    } else if (cmd.startsWith("CAL_PT ")) {
        // Format: CAL_PT <sensor> <raw> <real>
        int firstSpace = cmd.indexOf(' ', 7);
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        if (firstSpace > 0 && secondSpace > 0) {
            String sensor = cmd.substring(7, firstSpace);
            float raw = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            float real = cmd.substring(secondSpace + 1).toFloat();
            
            if (sensor == "pressure") pressureSensor.addCalibrationPoint(raw, real);
            else if (sensor == "voltage") voltageSensor.addCalibrationPoint(raw, real);
            else if (sensor == "current") currentSensor.addCalibrationPoint(raw, real);
            Serial.println("CAL_PT_ADDED");
        }
    } else if (cmd == "CAL_SAVE") {
        saveSystemConfig();
        Serial.println("CAL_SAVED");
    } else if (cmd.startsWith("SET_LIMITS ")) {
        // SET_LIMITS <uv> <oc> <sag>
        int firstSpace = cmd.indexOf(' ', 11);
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        if (firstSpace > 0 && secondSpace > 0) {
            float uv = cmd.substring(11, firstSpace).toFloat();
            float oc = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            uint8_t sag = cmd.substring(secondSpace + 1).toInt();
            
            SafetySettings s;
            s.minVoltage = uv;
            s.maxCurrent = oc;
            s.sagLimitPercent = sag;
            pumpMotor.setSafetySettings(s);
            saveSystemConfig(); // Also persist
            Serial.println("LIMITS_UPDATED");
        }
    }
}

void processSerial() {
    static char serialBuffer[64];
    static size_t bufferIndex = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                serialBuffer[bufferIndex] = '\0';
                String cmd = String(serialBuffer);
                cmd.trim();
                executeCommand(cmd);
                bufferIndex = 0;
            }
        } else {
            if (bufferIndex < sizeof(serialBuffer) - 1) {
                serialBuffer[bufferIndex++] = c;
            } else {
                bufferIndex = 0; // overflow prevention: reset buffer
            }
        }
    }
}

#ifdef ESP32
void processUDP() {
    int packetSize = udpClient.parsePacket();
    if (packetSize > 0) {
        char packetBuffer[64];
        int len = udpClient.read(packetBuffer, sizeof(packetBuffer) - 1);
        if (len > 0) {
            packetBuffer[len] = '\0';
            String cmd = String(packetBuffer);
            cmd.trim();
            executeCommand(cmd);
        }
    }
}
#endif

void sendTelemetry() {
    if (!isConnected) return; // Silent until handshake
    
    // Serial telemetry is throttled to 90Hz (every 11ms) to fit serial buffer limits of 115200 baud.
    if (millis() - lastTelemetry > 11) {
        lastTelemetry = millis();

        char payload[128];
#ifdef ESP32
        double ts = (double)esp_timer_get_time() / 1000.0;
#else
        double ts = (double)millis();
#endif
        snprintf(payload, sizeof(payload), "BIP,%.3f,%.2f,%d,%.2f,%.2f,%d,%.2f,%.2f,%.2f",
                 ts,
                 pressureSensor.getValue(),
                 pumpMotor.getCurrentPWM(),
                 voltageSensor.getValue(),
                 currentSensor.getValue(),
                 currentMode,
                 pressureSensor.getRawValue(),
                 voltageSensor.getRawValue(),
                 currentSensor.getRawValue());

        uint8_t checksum = 0;
        for (char* p = payload; *p; p++) {
            checksum ^= (uint8_t)*p;
        }

        char packet[160];
        snprintf(packet, sizeof(packet), "$%s*%02X\r\n", payload, checksum);
        Serial.print(packet);
    }
}

void loop() {
#ifndef ESP32
    wdt_reset();
#endif
    
    processSerial();
    #ifdef ESP32
    processUDP();
    #endif
    
    #ifdef ESP32
    if (wifiConnected && isConnected) {
        bool localBufferReady = false;
        bool localSendingBufferA = false;
        portENTER_CRITICAL(&timerMux);
        if (bufferReadyToSend) {
            localBufferReady = true;
            localSendingBufferA = sendingBufferA;
            bufferReadyToSend = false;
        }
        portEXIT_CRITICAL(&timerMux);

        if (localBufferReady) {
            TelemetrySample* sendBuffer = localSendingBufferA ? bufferA : bufferB;
            char packetBuffer[1500];
            int offset = 0;
            packetBuffer[0] = '\0';
        
        // Pack all buffer samples into a single UDP packet separated by newlines
        for (int k = 0; k < TELEM_BUFFER_SIZE; k++) {
            char payload[128];
            snprintf(payload, sizeof(payload), "BIP,%.3f,%.2f,%d,%.2f,%.2f,%d,%.2f,%.2f,%.2f",
                     sendBuffer[k].timestamp,
                     sendBuffer[k].pressure,
                     sendBuffer[k].pwm,
                     sendBuffer[k].voltage,
                     sendBuffer[k].current,
                     sendBuffer[k].mode,
                     sendBuffer[k].rawPressure,
                     sendBuffer[k].rawVoltage,
                     sendBuffer[k].rawCurrent);
                     
            uint8_t checksum = 0;
            for (char* p = payload; *p; p++) {
                checksum ^= (uint8_t)*p;
            }
            
            char line[160];
            int line_len = snprintf(line, sizeof(line), "$%s*%02X\n", payload, checksum);
            
            if (offset + line_len < sizeof(packetBuffer) - 1) {
                strcpy(packetBuffer + offset, line);
                offset += line_len;
            }
        }
        
        // Broadcast packet
        IPAddress broadcastIP;
        broadcastIP.fromString(UDP_BROADCAST_IP);
        udpClient.beginPacket(broadcastIP, UDP_PORT);
        udpClient.write((const uint8_t*)packetBuffer, offset);
        udpClient.endPacket();
    }
    #endif
    
    // Update Hardware
    bool readSensors = true;
    #ifdef ESP32
    if (wifiConnected && isConnected) {
        readSensors = false;
        
        // Handle DRDY processing in main loop
        if (drdyTriggered) {
            drdyTriggered = false;
            totalSamplesProcessed++;
            
            // Read 24-bit pressure value over SPI
            digitalWrite(PIN_ADS_CS, LOW);
            SPI.transfer(0x12); // RDATA command
            uint8_t b0 = SPI.transfer(0x00);
            uint8_t b1 = SPI.transfer(0x00);
            uint8_t b2 = SPI.transfer(0x00);
            digitalWrite(PIN_ADS_CS, HIGH);
            
            int32_t raw_val = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | b2;
            if (raw_val & 0x800000) {
                raw_val |= 0xFF000000; // Sign-extend 24-bit to 32-bit
            }
            
            adsRawPressure = raw_val;
            
            int rawV = analogRead(PIN_VOLTAGE_SENSOR);
            int rawI = analogRead(PIN_CURRENT_SENSOR);
            
            float volts_val = voltageSensor.mapMultiPoint(rawV * (V_REF / ADC_MAX_VAL));
            float current_val = currentSensor.mapMultiPoint(rawI * (V_REF / ADC_MAX_VAL));
            
            float press_volts = (float)raw_val * (V_REF / 8388607.0f);
            pressureSensor.setRawValue(press_volts);
            float pressure_val = pressureSensor.getValue();
            
            TelemetrySample* activeBuffer = useBufferA ? bufferA : bufferB;
            
            activeBuffer[writeIndex].timestamp = (double)esp_timer_get_time() / 1000.0;
            activeBuffer[writeIndex].pressure = pressure_val;
            activeBuffer[writeIndex].pwm = pumpMotor.getCurrentPWM();
            activeBuffer[writeIndex].voltage = volts_val;
            activeBuffer[writeIndex].current = current_val;
            activeBuffer[writeIndex].mode = currentMode;
            activeBuffer[writeIndex].rawPressure = (float)raw_val;
            activeBuffer[writeIndex].rawVoltage = (float)rawV;
            activeBuffer[writeIndex].rawCurrent = (float)rawI;
            
            writeIndex++;
            if (writeIndex >= TELEM_BUFFER_SIZE) {
                portENTER_CRITICAL(&timerMux);
                sendingBufferA = useBufferA;
                useBufferA = !useBufferA;
                writeIndex = 0;
                bufferReadyToSend = true; // Signals main thread to transmit
                portEXIT_CRITICAL(&timerMux);
            }
        }
    }
    #endif
    if (readSensors) {
        updateSensors();
    }
    
    userInput.update();
    
    // Safety check passing sensor values
    pumpMotor.update(currentSensor.getValue(), voltageSensor.getValue(), 12.0); 
    
    // Check if Motor triggered a Safety Lockout
    if (pumpMotor.isSafetyTriggered() && currentMode != MODE_ERROR) {
        triggerError("MOTOR SAFETY", pumpMotor.getSafetyError());
    }
    
    // Logic
    updateStateMachine();
    
    // Telemetry (For Serial output)
    sendTelemetry();
    
    delay(LOOP_INTERVAL_MS);
}
