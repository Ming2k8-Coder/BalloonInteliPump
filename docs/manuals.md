# Balloon Intelligent Pump (BIP) - System Operations Manual

The **Balloon Intelligent Pump (BIP)** is a high-precision, industrial-grade automated telemetry system engineered to analyze the mechanical and viscoelastic properties of latex balloons and inflatable elastomeric materials.

---

## 🚀 1. Firmware Platform Architecture

### ESP-IDF Native Framework (`espidf` branch)
- **Real-Time DSP Engine:** Core 1 dedicated task running high-speed SPI 2000 SPS ADS1220 ADC sampling with microsecond DRDY interrupt notifications.
- **Async Networking Stack:** Core 0 dedicated task running ESP-IDF SoftAP WiFi stack, 32KB FreeRTOS lock-free RingBuffer (`ringbuf.h`), and LwIP BSD UDP sockets.
- **Hardware PWM:** 25 kHz ultrasonic motor drive and 20 kHz solenoid valve drive via ESP32 LEDC timers.
- **Persistent Storage:** Non-Volatile Storage (NVS) for calibration tables and safety settings.

---

## 💻 2. PC Dashboard & Telemetry Backend

The system features two connection interfaces:
- **WiFi UDP Broadcast (Port 8888):** High-speed 100Hz packet streaming over LwIP BSD UDP sockets.
- **USB Serial NMEA (115200 Baud):** Standard serial connection.

### Launching the System
```bash
# 1. Activate Python virtual environment
source .venv/bin/activate  # Or .venv\Scripts\Activate.ps1

# 2. Launch HTTP SSE Server & Time-Series RLS Predictor
python server/app.py

# 3. Launch Desktop GUI Controller
python gui/app.py
```

---

## 🛠️ 3. Operation Modes

### 🟢 Manual Mode & Target Pressure Hold
- **Direct PWM Control:** Set pump duty cycle directly (`0–255`).
- **Closed-Loop PID Target Hold:** Enter a target pressure (e.g. `20.0 kPa`). The PID controller dynamically modulates pump PWM output with anti-windup clamping and D-kick derivative filtering.

### 🔵 Smart Mode (Yield Point Detection)
1. Configure **Yield Ratio** (Default: `1.20`).
2. Click **Start Smart Test**.
3. The system monitors the pressure gradient ($dP/dt$) at 2000 SPS, flags the first physical expansion (Yield Peak), and inflates to `YieldPeak * Ratio` before shutting off.

### 🔴 Burst Mode (Destructive Testing)
- **Caution:** Wear eye protection and operate within a protective enclosure.
- Click **Start Burst Test**. The pump drives at 100% capacity until sample failure, capturing peak destruction pressure.

---

## 🧪 4. Bare-MCU Unity Unit Testing

You can run full automated unit testing on a bare ESP32 board over standard USB-UART without connecting physical sensors or pumps.

### Executing Tests
```bash
# Flash and run Unity Test Suite over standard USB-UART
idf.py -p COM_PORT test monitor
```

### Verified Test Categories
1. **`test_bip_sensors`:** Validates Tare zeroing, multipoint linear mapping, and zero-division guards.
2. **`test_bip_motor`:** Validates soft-start PWM ramping, 3-sample inrush filter, and UV lockout.
3. **`test_bip_state`:** Validates PID error integration, derivative filtering, and yield state transitions.
4. **`test_bip_storage`:** Validates NVS struct serialization.

---

## ⚖️ 5. Calibration & Zero-Tare Guidelines

1. **Zero-Tare:** Send `ZERO_PRESSURE` to zero the ambient baseline prior to inflation.
2. **Multipoint Calibration:** Use the Desktop Calibration Wizard (`gui/app.py`) to map raw ADC voltages to physical unit measurements ($kPa$, $V$, $A$).
3. **Commiting to NVS:** Click **Save Permanently** to persist calibration values to ESP32 NVS flash storage.

---

## 🛡️ 6. Hardware Safety Protocols

- **Undervoltage Lockout (`UV_LOCK`):** Trips motor off if battery drops below `10.5V`.
- **Overcurrent Filter (`OC_LOCK`):** Evaluates motor current against `4.5A` max, using a 3-sample inrush filter to avoid false trips during motor turn-on surges.
- **Voltage Sag Ratcheting:** Automatically lowers PWM duty cycle during heavy battery sag to stabilize power rails.
- **Predictive Cutoff:** RLS time-series predictor automatically halts inflation if 150ms forecasted pressure exceeds 95% of estimated burst limit.
