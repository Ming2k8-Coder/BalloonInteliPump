# Balloon Intelligent Pump (BIP) - User Manual

The **Balloon Intelligent Pump** is a high-precision testing system designed to measure and record the mechanical properties of balloons, specifically "First Yield" and "Burst Pressure."

---

## 💻 1. PC Dashboard Setup

The PC Dashboard (`gui/app.py`) is the primary interface for visualization and advanced control.

### Connection

1. Connect the Arduino to your PC via USB.
2. Launch the app: `python gui/app.py`.
3. Select the correct **COM Port** and click **Connect**.
4. The status should change to `CONNECTED`, and telemetry data (Pressure, Voltage, Current) will begin streaming at 100Hz.

### Data Logging

- Click **Start Log** before beginning a test to record every data point to a CSV file.
- The log includes both **Calibrated Values** (kPa, V, A) and **Raw ADC Values** (for post-test verification).

---

## 🛠️ 2. Operation Modes

### 🟢 Manual Mode

Used for system checkouts or specific inflation levels.

- **PWM Control**: Slide the PWM bar to adjust pump speed (0-255).
- **Target Pressure**: Enter a value (e.g., `15.0`) and click **Set Target Pressure**. The pump will automatically cycle to maintain this pressure.

### 🔵 Smart Mode (Yield Detection)

The "Flagship" mode of the BIP.

1. Enter the **Yield Ratio** (Default: `1.2`). This tells the system to stop at 120% of the pressure found at the first physical expansion (Yield).
2. Click **Start Smart Test**.
3. The pump will inflate until it detects a momentary pressure drop (The Yield Peak).
4. After detection, it continues to `YieldPeak * Ratio` and then shuts off automatically.

### 🔴 Burst Mode

**CAUTION: Wear eye protection and use an enclosure.**

- Click **Start Burst Test**.
- The pump runs at full power until the balloon ruptures.
- The system captures the absolute **Peak Pressure** recorded just before the pop.

---

## ⚖️ 3. Calibration Manual

Consistent results require accurate calibration. The BIP supports **Multipoint Linear Interpolation**.

### Using the PC Calibration Wizard

1. Open **Calib Tab** -> **Manual Calib Wizard**.
2. **Select Sensor**: Choose Pressure, Voltage, or Current.
3. **Capture Data**:
    - Apply a known physical load (e.g., Use a manual pump with a gauge to apply 20kPa).
    - Click **Use Current Raw** to grab the real-time ADC value from the sensor.
    - Enter the **Real Value** (e.g., `20.0`) in the box.
    - Click **Add Calibration Point**.
4. **Repeat**: Add at least 2 points (e.g., 0kPa and 50kPa).
5. **Finalize**: Click **SAVE PERMANENTLY TO EEPROM**.

### BMP280 Self-Calibration

If you have a BMP280/BME280 sensor installed inside your test chamber:

1. Go to the **Calib** tab.
2. Click **BMP280 Self-Calib (Ref)**.
3. The Arduino will use the high-accuracy BMP280 as a reference to calibrate your primary Analog pressure sensor automatically.

---

## 🛡️ 4. Safety Systems

The system monitors hardware health in real-time. If a safety limit is hit, the pump stops and the LCD/PC shows an error.

- **Under-voltage (UV)**: Protects your LiPo/Power supply from over-discharge.
- **Over-current (OC)**: Protects the MOSFET and Pump motor from stalls or shorts.
- **Sag Protection**: If the voltage drops significantly while running, the system will "Ratchet down" the PWM speed to stabilize the power rail rather than cutting out entirely.

*Tuning these limits can be done via the **Limits** tab in the PC App.*

---

## 📋 5. Troubleshooting

| Issue | Likely Cause | Solution |
| :--- | :--- | :--- |
| **Serial Error** | Cable noise or Buffer overflow | Check USB cable; Ensure `BAUD_RATE` is 115200. |
| **Pump doesn't start** | Safety Lockout | Check Voltage/Current limits in the "Limits" tab. |
| **Noisy Pressure** | Electrical Interference | Add 100nF capacitors across the motor terminals. |
| **Wrong Pressure** | Outdated Calib | Run the Calibration Wizard. |
