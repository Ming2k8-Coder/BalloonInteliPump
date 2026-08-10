# BIP Project Context Transfer Document

This document provides a comprehensive architectural and engineering overview of the **Balloon Intelligent Pump (BIP)** system. It is designed to quickly bootstrap any large language model (particularly the Gemini LLM family) with the full project context, architecture, pinouts, API specifications, and database structures.

---

## 1. System Overview
BIP is a modular cyber-physical system designed to inflation-test, calibrate, and characterize the mechanical and viscoelastic properties of latex balloons. It operates as a dual-part system:
*   **Edge Device (ESP32 / Arduino Nano):** Captures high-rate ADC values from a 24-bit ADS1220 ADC or local analog inputs, executes a closed-loop PID controller, runs motor safety limit checks, and coordinates the hardware state machine.
*   **Host Server / Dashboard (Python Web/GUI):** Receives 2kSPS double-buffered UDP telemetry packets (or 100Hz serial logs), calculates rolling digital signal processing (DSP) features, runs a local Machine Learning (Random Forest) classifier for material state analysis, interfaces with local LLMs (Ollama/llama.cpp) for supervisory planning, and logs full-precision datasets.

---

## 2. Hardware Architecture & Pin Mapping

### Microcontroller Configurations (ESP32 vs. Arduino Nano)
The code features `#ifdef ESP32` toggles to support both platforms, but high-speed WiFi double-buffering is exclusive to the ESP32:

*   **SPI ADS1220 ADC Connections:**
    *   `PIN_ADS_CS` = GPIO 5 (Chip Select)
    *   `PIN_ADS_DRDY` = GPIO 17 (Data Ready Interrupt input)
    *   `SPI Bus` = Default hardware SPI pins (SCLK=18, MISO=19, MOSI=23 on ESP32)
*   **Actuator Control Outputs:**
    *   `PIN_PUMP_PWM` = GPIO 9 (Logic-level gate drive for the main 12V pump MOSFET)
    *   `PIN_SOLENOID_VALVE` = GPIO 12 (Gate drive for the 12V relief/venting valve MOSFET)
*   **Local Sensors (ESP32 ADC fallbacks):**
    *   `PIN_PRESSURE_SENSOR` = Analog channel A0
    *   `PIN_VOLTAGE_SENSOR` = Analog channel A1 (Battery monitor via resistor divider)
    *   `PIN_CURRENT_SENSOR` = Analog channel A2 (ACS712 current sensor)

---

## 3. Firmware State Machine & Timing

### Dual Buffer Telemetry Pipeline (2kSPS UDP)
When configured with WiFi, the ESP32 bypasses the slow loop timer:
1.  **Hardware Interrupted Reading:** The ADS1220 DRDY pin triggers a falling-edge hardware interrupt `ads1220_isr()` exactly 2000 times per second ($500\,\mu\text{s}$ interval).
2.  **SPI RDATA Retrieval:** The ISR pulls CS low, sends command `0x12` (Read Data), and shifts in 3 bytes (24-bit sign-extended format).
3.  **Atomic Buffer Swapping:** Telemetry samples are packed into `bufferA` or `bufferB` (10 samples per buffer). When a buffer fills, pointers swap atomically and a `bufferReadyToSend` flag is raised to let the Core 1 main loop transmit the packet over UDP.

### Operational Modes (`ModeCode`)
*   **`0` - IDLE:** Motor off, valve closed, awaiting start instructions.
*   **`1` - MANUAL:** Closed-loop PID target hold or direct target PWM output.
*   **`2` - SMART:** Monitored yield point detection (identifies elastomeric boundary).
*   **`3` - BURST:** Continuous 100% duty cycle inflation until rupture.
*   **`4` - CALIB:** Calibrating multipoint sensors in real time.
*   **`5` - ERROR:** Watchdog or safety lockouts active (under-voltage, over-current).

---

## 4. Host API & Telemetry Framing

### UDP/Serial Telemetry Framing
Each CSV payload starts with a `$` character and ends with a 2-digit hex XOR checksum:
```
$BIP,Time(ms),Pressure(kPa),CurrentPWM,Voltage(V),Current(A),ModeCode,RawPress,RawVolt,RawCurrent*Checksum
```

### Serial/UDP Command Set
The edge firmware parses the following line-terminated ASCII instructions:
*   `CONNECT` - Establish handshake and start active output stream.
*   `STOP` - Immediate hardware shutdown (pump = 0, safety lock).
*   `VALVE_ON` / `VALVE_OFF` - Vent or seal the solenoid relief valve.
*   `SET_VALVE_PWM <0-255>` - Pulse width modulate the solenoid (flow/holding-current regulation).
*   `SET_PWM <0-255>` - Sets direct pump speed.
*   `SET_TARGET <kPa>` - Updates target pressure for the PID controller.
*   `START_MANUAL` / `START_SMART` / `START_BURST` - Transitions system states.

---

## 5. Machine Learning & LLM Integration

### Scikit-Learn Random Forest Classifier
*   **Rolling DSP Features:** The server processes raw telemetry to generate online features:
    *   `Pressure_Var_300ms`: Rolling pressure variance (detects squeezes/vibrations).
    *   `Pressure_Integral_500ms`: Area under pressure curve (cumulative material stress).
    *   `Power_W`: Current pump wattage.
    *   `Power_to_Pressure_Ratio`: Tracks pump output efficiency vs load.
    *   `Creep_Slope`: Rates decay during static hold states when the pump is off.
*   **States Predicted:** `STABLE` (0), `SQUEEZE` (1), `YIELDING` (2), `DANGER` (3).

### Local LLM Custom Driver (`LoonerPleasureDriver`)
A background thread periodically compiles rolling statistics and formats a JSON-producing prompt sent to local `Ollama` or `llama.cpp` instances running on the host GPU (AMD RX 5600XT).
*   **Interactive Mode:** Automatically vents the solenoid valve if a user squeeze or pop danger is classified by the ML.
*   **Massage Mode:** Rhythmical sinuous pressure waves (+/- 1.5 kPa at 0.08 Hz) are modulated to create physical vibration.
*   **LLM Mode:** Asynchronously alters targets and relief cycles using LLM reasoning.
