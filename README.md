# Balloon Intelligent Pump (BIP)

BIP is a modular, high-precision automated testing and measurement system designed to analyze the mechanical and viscoelastic properties of latex balloons. Featuring smart yield-point detection, burst testing, real-time telemetry, and PC-connected analysis, BIP bridges the gap between simple inflation and scientific material characterization.

---

## 🚀 Key Features

### 🟢 Intelligent Operation Modes

- **Smart Inflation (Yield Detection)**: Automatically monitors the pressure gradient, flags the balloon's first material expansion (yield point), and safely inflates to a configurable multiplier (e.g., `Yield Pressure × 1.20`) before stopping.
- **Controlled Burst Test**: Inflates a sample at maximum output until failure occurs, capturing the exact peak destruction pressure.
- **Manual Mode with Closed-Loop Target Hold**: Directly set the pump's PWM power or define a target pressure that the system dynamically maintains.

### ⚖️ Calibration & Sensors

- **Universal Multipoint Calibration**: Supports up to 5-point calibration mappings for pressure, current, and voltage sensors to correct non-linearities.
- **BMP280/BME280 Reference Auto-Calib**: Optional reference barometric sensor allows for single-click automatic calibration of the primary analog pressure sensor.
- **Flexibility**: Works with both low-pressure sensors (MPS20N0040D-S, 0–40kPa) and high-pressure sensors (XGZP6847A, 0–200kPa).

### 🛡️ Hardware-Level Safety Systems

- **Watchdog Timer (WDT)**: Integrated 2-second watchdog prevents runaway states in case of MCU freeze.
- **Overcurrent & Undervoltage Lockout**: Immediately halts the motor if voltage drops below a safe threshold (protecting battery chemistry) or current exceeds set parameters.
- **Voltage Sag Protection**: A ratcheting PWM limitation system dynamically lowers motor power during high-load voltage sags to prevent battery brownouts without shutting off.

### 💻 Real-Time PC Dashboard & Visualization

- **100Hz Telemetry Stream**: Connect via USB serial to display and plot live pressure data.
- **Graphical Calibration Wizard**: A desktop GUI assistant to capture raw ADC values and push calibrated parameters directly to the Arduino's EEPROM.
- **Data Logging**: Write structured CSV logs with metadata headers for external analysis (MATLAB, Origin, Excel).

---

## 📂 Project Structure

- [BalloonInteliPump/](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/): Modular Arduino firmware.
  - [Config.h](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/Config.h): Central configurations, safety limits, pinouts, and sensor selection.
  - [Sensors.cpp](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/Sensors.cpp) / [Sensors.h](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/Sensors.h): Multipoint calibration calculations and drivers.
  - [Motor.cpp](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/Motor.cpp) / [Motor.h](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/Motor.h): Soft-start ramping, voltage sag protection, and safety checks.
  - [StateFunctions.cpp](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/StateFunctions.cpp): Central state machine.
- [gui/](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/gui/): Python-based Tkinter desktop control panel.
  - [app.py](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/gui/app.py): Serial manager, dashboard UI, plotting, and Calibration Wizard.
- [docs/](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/docs/): Hardware schemas and documentation.
  - [BOM.md](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/docs/BOM.md): Complete list of mechanical and electrical components.
  - [Wiring.md](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/docs/Wiring.md): Circuit connections and schematics.
  - [manuals.md](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/docs/manuals.md): Comprehensive operational guidelines.
  - [ideas.md](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/docs/ideas.md): Planned improvements, scientific research ideas, and future roadmap.

---

## 🛠️ Setup & Installation

### 1. Hardware Assembly

Assemble the physical controller following the connections outlined in the [Wiring Guide](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/docs/Wiring.md). Ensure the 12V motor supply and the 5V Arduino Nano supply share a common ground.

### 2. Uploading the Firmware

1. Open the Arduino IDE.
2. Install the following libraries:
   - `LiquidCrystal_I2C` (for the local 16x2 screen)
   - `Adafruit_BMP280` (only if utilizing the auto-calibration reference sensor)
3. Open [BalloonInteliPump.ino](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/BalloonInteliPump.ino).
4. Review configurations in [Config.h](file:///e:/Master_Project/Coder_projext/BalloonInteliPump/Config.h). Select your input device (`USE_ROTARY_ENCODER` vs. 4x4 Keypad) and sensor type.
5. Upload the code to your Arduino Nano or Uno.

### 3. Running the Python GUI App

1. Set up a Python environment (Python 3.10+ recommended).
2. Install the required dependencies:

   ```bash
   pip install -r gui/requirements.txt
   ```

3. Launch the controller application:

   ```bash
   python gui/app.py
   ```

4. Choose your port from the dropdown menu, click **Connect**, and begin operation.

---

## 🎮 Command Interface API

When connected via USB Serial, the Arduino parses the following line-terminated commands:

- `CONNECT`: Initializes the PC connection and starts the telemetry output.
- `START_MANUAL`: Activates Manual mode (idle pump, ready for instructions).
- `START_SMART`: Launches the Yield-Detection automated inflation sequence.
- `START_BURST`: Runs the pump at 100% capacity until balloon burst is registered.
- `STOP`: Emergency stop. Immediately kills motor output and returns to the Idle state.
- `SET_PWM <0-255>`: Sets a direct motor power level (disables target hold).
- `SET_TARGET <pressure_kPa>`: Sets a manual target pressure for the closed-loop hold routine.
- `SET_PID <Kp> <Ki> <Kd>`: Dynamically adjusts PID controller gain parameters.
- `VALVE_ON` / `VALVE_OFF`: Fully opens (100%) or closes (0%) the solenoid relief valve.
- `SET_VALVE_PWM <0-255>`: Sets solenoid duty cycle for holding current / flow rate control.
- `SET_RATIO <ratio>`: Updates the Smart inflation yield threshold factor (e.g., `1.2`).
- `SET_LIMITS <uv_volts> <oc_amps> <sag_percent>`: Adjust safety threshold limits on-the-fly.
- `CAL_PT <sensor> <raw_val> <real_val>`: Inputs a raw vs. real calibration point. Valid sensors: `pressure`, `voltage`, `current`.
- `CAL_CLR <sensor>`: Resets all stored calibration points for the designated sensor.
- `CAL_SAVE`: Writes all added calibration points permanently into the Arduino's EEPROM slots.

---

## 📊 Telemetry Format

BIP outputs framed, checksummed CSV telemetry strings at 100Hz:

```csv
$BIP,Time(ms),Pressure(kPa),CurrentPWM,Voltage(V),Current(A),ModeCode,RawPressADC,RawVoltADC,RawCurrentADC*Checksum
```

- **Formatting details**:
  - Starts with `$` prefix.
  - Comma-separated telemetry payload starting with `BIP`.
  - Ends with `*` followed by a two-digit hexadecimal XOR checksum of the payload characters (all characters between `$` and `*`).
- **ModeCode Key**:
  - `0`: Idle State
  - `1`: Manual Mode
  - `2`: Smart Mode
  - `3`: Burst Mode
  - `4`: Calibration Mode
  - `5`: Error State
