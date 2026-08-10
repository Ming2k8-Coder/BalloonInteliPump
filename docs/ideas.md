# BIP - Future Ideas & Roadmap

A collection of improvement ideas and TODOs for the Balloon Intelligent Pump (BIP) project, reorganized by development priority and hardware dependencies.

---

## 🔴 High Priority: Core Firmware & Safety Updates (Existing Hardware)

*Software-only enhancements utilizing the current sensor suite (analog pressure, ACS712 current, voltage divider) and output (12V pump motor via MOSFET, LCD, USB serial).*

### 1. PID Controller for Target Pressure

**Problem**: Manual target mode currently uses simple bang-bang control (on/off at PWM 150), which causes pressure overshoots and constant oscillation.

- **Solution**: Implement a closed-loop Proportional-Integral-Derivative (PID) controller.
- Add configurable `Kp`, `Ki`, `Kd` tuning parameters in `Config.h` with sensible defaults.
- Support live tuning via serial commands: `SET_PID <Kp> <Ki> <Kd>`.
- Add a dedicated PID tuning tab to the Python GUI app.

### 2. Smarter Yield Detection (dP/dt Slope)

**Problem**: The current heuristic relies on a hardcoded 2kPa pressure drop from a peak, failing on thick-skinned balloons or noisy sensors.

- **Solution**: Switch to a **sliding window derivative** (dP/dt) algorithm.
- Identify the yield point when the slope (first derivative) flattens or goes negative for more than `N` consecutive samples.
- Implement a configurable threshold command: `SET_YIELD_SENS <value>`.
- Add a software moving average filter to clean up raw sensor inputs.

### 3. Neck Inflation Detection (Near-Max Warning)

- **Concept**: The neck of the balloon beginning to inflate is the clearest physical sign that it is reaching its max size limit.
- **Method**: Detect the pressure plateau or slight dip (`dP/dt` going near zero at high pressures) during active inflation.
- **Action**: Instantly throttle the pump to a slow crawl speed, trigger a warning flag in the telemetry stream, and stop inflation after a small safety margin: `SET_NECK_MARGIN <kPa>`.

### 4. Slow-Approach Sizing Algorithm

- **Concept**: Slow down pump speed in stages as the balloon approaches the target pressure to prevent accidental bursts.
- **Zones**:
  1. *Fast Zone* (0–70% of target): Inflate at full PWM.
  2. *Caution Zone* (70–90% of target): Drop to 50% PWM.
  3. *Crawl Zone* (90–100% of target): Drop to minimum pump PWM (30-50).
- Configurable zone boundaries: `SET_ZONES <fast%> <caution%>`.

### 5. Elastic Reserve Monitoring

- **Concept**: Track how much stretch capacity remains before failure.
- **Calculation**: `Elastic Reserve % = (Burst Pressure - Current Pressure) / Burst Pressure × 100%`.
- **Sweet Spot**: Typically **35–50% reserve** for safe looner play (sitting/bouncing/squeezing).
- **UI**: Display live reserve percentage on both the LCD screen (`RES: 42%`) and the PC App Dashboard.

### 6. Active Play Monitoring Mode (`MODE_PLAY`)

- **Concept**: A dedicated mode to monitor and log pressure while someone actively plays with the balloon while it remains connected via a long tube.
- **Logic**: Pump motor stays disabled (or runs micro-puffs to compensate for micro-leaks).
- Stream telemetry at 100Hz to accurately record squeeze and impact spikes.
- Serial command: `START_PLAY`.

### 7. Dynamic Impact Peak Detection

- **Concept**: Capture and log transient spikes from active play (squeezes, bounces, body weight).
- **Algorithm**: Detect fast pressure rises (`dP/dt > threshold`).
- **Telemetry**: Log absolute peak pressure, spike duration, and rise time of each play impact.

### 8. Play-Pop Predictor Alert

- **Concept**: Warn the player when their physical interaction forces the balloon too close to its limit.
- **Trigger**: If a dynamic squeeze spike reaches within 10% of the known burst pressure, flash the LCD backlight, beep, and flag a warning in the telemetry.

### 9. Auto-Top-Up (Micro-Leak Compensator)

- **Concept**: Latex naturally leaks air under high tension.
- **Method**: If baseline pressure drops below the target by >5% when not actively being squeezed, the pump runs short "micro-puffs" (e.g., 50ms at PWM 40) to top up the balloon without interrupting play.

### 10. Long-Tube Pressure Delay Calibration

- **Concept**: A long tether tube dampens pressure changes and introduces latency between the balloon and the sensor.
- **Solution**: A latency calibration routine. Measure the time delay of a pulse to estimate the tube's resistance, then apply an offset algorithm: `P_balloon = P_sensor + K_dampening * (dP/dt)`.

---

## 🟡 Medium Priority: Advanced Data & Analysis (Existing Hardware)

*Features that enhance research capabilities, data logging, and PC GUI visualization without adding new physical components.*

### 11. Multi-Plot PC Dashboard

- **Telemetry expansion**: Add subplots to the Python GUI to plot Current vs. Time and Voltage vs. Time alongside Pressure.
- Support dual-axis plots (Pressure and PWM on same graph) and a zoom-to-region tool for post-test analysis.

### 12. Test Session & Batch Management

- **Run Counter**: Automatically increment test numbers.
- **CSV Metadata**: Save headers with date, sensor type, and calibration points.
- **Export Summary**: Generate PDF/HTML reports of completed test batches.

### 13. Temperature Compensation (BMP280)

- Leverage the BMP280's built-in temperature sensor to compensate for pressure sensor thermal drift.
- Log temperature in telemetry and display it on the LCD during idle state.

### 14. Stress-Strain Curve Generation

- **Method**: Use a thin-wall sphere model to estimate wall stress: `σ = (P × r) / (2 × t)`.
- Estimate strain (`ε`) from the volume change (calculated via flow estimation or pressure models).
- Export stress-strain datasets directly to CSV.

### 15. Statistical Batch Testing Tools

- Auto-calculate batch metrics: Mean, Median, Standard Deviation, and Coeff of Variation.
- Calculate **Weibull Distribution** fit (standard for material failure analysis) and 95% confidence intervals.

### 16. Hysteresis & Viscoelasticity Models

- Track pressure decay under constant stretch to characterize **Creep & Stress Relaxation**.
- Fit logged data to Maxwell or Kelvin-Voigt viscoelastic models in the PC app.

### 17. Real-Time Size Estimation from Pressure

- **Concept**: Use the Mooney-Rivlin hyperelastic model for rubber inflation.
- Solve stretch ratio `λ = r/r₀` using measured pressure and material constants to estimate balloon diameter in real-time without external sensors.

### 18. Wall Thickness Estimation

- Estimate latex wall thinning during inflation using volume conservation: `t = t₀ × (r₀/r)²`.
- Display estimated wall thickness (e.g., `WALL: 0.08mm`) on screen.

### 19. Community Lookup Database

- Integrate a local JSON database in the PC app containing pre-tested brand parameters (Qualatex, Belbal, Tuftex, etc.) including burst averages, target play pressures, and safety limits.

### 20. Squeeze Frequency Analysis (FFT)

- **Concept**: Use Fast Fourier Transform (FFT) on the 100Hz pressure data to analyze rhythmic bouncing or squeezing patterns.
- **Output**: Display the dominant frequency (Hz) of the player's interactions (e.g., "Bounce Rate: 2.5 Hz").

### 21. Total Energy Absorption (Integral)

- **Concept**: Calculate the integral (area under the curve) of pressure spikes over time during the play session.
- **Output**: Quantifies how much total physical work/energy the balloon has absorbed during the session, which correlates heavily to latex fatigue.

### 22. Baseline Creep Tracking

- **Concept**: Track the "resting pressure" between squeezes. Latex stretches permanently under stress.
- **Output**: As the resting pressure drops over a long session, the software charts the physical degradation of the balloon and estimates remaining play lifespan.

### 23. Micro-Movement & Respiration Tracking

- **Concept**: High-resolution ADCs (like the ADS1220) are sensitive enough to pick up micro-fluctuations.
- **Output**: Can detect a person breathing, heartbeat, or slightly shifting their weight while resting on a large balloon.

### 24. Time-Under-Tension (TUT) Fatigue Tracker

- **Concept**: Track the cumulative number of seconds the balloon spends squeezed above 80% of its burst threshold.
- **Output**: Warns the player when the balloon has exceeded safe fatigue limits based on total time squeezed, not just peak pressure.

### 25. Dynamic Deflation Relief (Lifespan & Comfort)

- **Concept**: If the system detects a sustained high-pressure squeeze (e.g., the user sits fully on the balloon and stays there), the latex enters high-fatigue mode.
- **Output**: Automatically vents a micro-burst of air (using the solenoid valve) to bring the pressure down to the "comfort/safe zone". This prevents fatigue tearing and provides a softer "give" yielding sensation to the user.

### 26. Adaptive Safety Margin (Lifespan)

- **Concept**: The longer a play session goes on, the more the latex micro-tears and degrades. A pressure that was safe at minute 5 might pop the balloon at minute 45.
- **Output**: The system automatically lowers the "Maximum Safe Target Pressure" dynamically based on the total time-under-tension (e.g., reducing the safety ceiling by 1% for every 5 minutes of heavy play) to completely eliminate surprise fatigue pops.

### 27. Softness "Sweet Spot" Profiling (Pleasure)

- **Concept**: At the beginning of play, the system runs a quick `+2 kPa / -2 kPa` micro-inflation test to calculate the stiffness curve (Young's modulus) of the current balloon.
- **Output**: It calculates and sets the pump to the exact "Softness Sweet Spot"—the highest pressure where the balloon is massive but still feels soft, squishy, and highly responsive to touch, rather than feeling rigid and drum-tight.

### 28. Thermal Degradation Tracking (Lifespan)

- **Concept**: Aggressive bouncing and pumping heats up the latex. Hot latex is much more prone to yielding and permanent deformation (creep).
- **Output**: Use the temperature probe data to calculate a "Heat Fatigue Index". If the balloon gets too warm during aggressive play, it alerts the user to pause and let the rubber cool down, massively extending the session life.

### 29. Rhythmic Pulsing/Massage Mode (Pleasure)

- **Concept**: Instead of holding a constant static pressure, leverage the PID controller and solenoid valve to create a rhythmic pressure wave.
- **Output**: The balloon smoothly inflates and deflates slightly (e.g., varying by 1 kPa at 0.5 Hz). This creates a tactile "breathing" or massaging sensation for the user resting against it.

---

## 🟢 Low Priority: Convenience & UI Tweaks (Existing Hardware)

*Minor software features and UI updates to improve device interaction.*

### 30. Leak Detection Mode (`MODE_LEAK`)

- Inflate to target pressure, shut off the pump, and monitor the decay slope over time to report the leakage rate in kPa/min.

### 31. Flow Rate Estimation (Inferred)

- Estimate flow rate and balloon volume by integrating pressure rise rate (`dP/dt`) during constant-PWM inflation.

### 32. Custom LCD Characters & Animation

- Use `lcd.createChar()` to show a horizontal pressure bar graph, up/down trend arrows, a battery level indicator, and a spinning pump animation.

### 33. Play Session Leaderboard

- A local scoreboard tracking the "Strongest Squeeze" (highest non-burst spike), "Longest Play Session" (time under tension), and "Toughest Balloon".

---

## 🔵 Very Low Priority: Hardware Expansions (Requires New Modules/Sensors)

*Concepts that require purchasing and integrating additional hardware modules, sensors, or relays.*

### 34. Solenoid Valve Support (Deflation & Active Cycling)

- **Requires**: 12V Solenoid valve, secondary driving MOSFET, and flyback diode.
- **Enables**: Active deflation, cycling tests (fatigue cycles), and controlled pressure releases in Manual/Smart modes.

### 35. Size-Based Sizing Sensors

- **Requires**: HC-SR04 Ultrasonic distance sensor or VL53L0X Laser distance sensor (I2C).
- **Enables**: Target inflation to exact physical dimensions (e.g., stop at exactly 35cm diameter).

### 36. Audible Feedback Buzzer

- **Requires**: Passive/Active piezo buzzer.
- **Enables**: Alarms for safety lockouts, burst alerts, yield indicators, and play warning tones.

~~### 37. Wireless Connectivity Upgrades~~

- ~~**Requires**: *Covered by transitioning to ESP32.*~~
- ~~**Enables**: Wireless telemetry streaming, OTA updates, and running a web-based dashboard directly on the ESP32.~~

### 38. SD Card Logging (Standalone Mode)

- **Requires**: SPI SD Card Module.
- **Enables**: Standalone logging of CSV files when disconnected from a PC, dumpable over Serial later.

### 39. OLED Display Upgrade

- **Requires**: SSD1306 128x64 OLED Display (I2C).
- **Enables**: Real-time graphing directly on the device, richer UI menus, and battery/connection indicators.

### 40. Multiple Sensor Channels

- **Requires**: CD4051 Analog Multiplexer and extra pressure sensors.
- **Enables**: Testing 2–4 balloons simultaneously on the same hardware.

### 41. Strain Gauge Integration

- **Requires**: HX711 Amplifier module and a stretch sensor attached to the balloon.
- **Enables**: Measuring direct material stretch/deformation to generate true stress-strain curves.

### 42. Multi-Stage Conditioning Inflation

- **Requires**: Solenoid valve (Idea #34).
- **Enables**: Automating pre-stretch inflate-hold-deflate cycles, letting polymer chains rearrange to make the balloon physically larger and stronger at play pressure.

### 43. Fatigue & Cycle Testing

- **Requires**: Solenoid valve (Idea #34).
- **Enables**: Automating lifecycle tests (cycle inflation to X kPa → deflate → repeat N times) to research latex lifespan.

### 44. Environmental Chamber Control

- **Requires**: Humidity sensor (DHT22/SHT31), solid-state relay, small heater, and insulated box.
- **Enables**: Running automated tests under controlled temperature/humidity zones.

### 45. Differential Pressure Configuration

- **Requires**: Second analog pressure sensor.
- **Enables**: Differential pressure measurements when inflating a balloon inside a vacuum or pressurized chamber.

---

## 🐛 Known Issues & Bug Fixes (High Priority)

- **Bang-bang oscillation**: Manual target mode overshoots pressure targets. (*Fix with PID control, Idea #1*).
- **LCD flicker**: `lcd.clear()` inside state updates causes annoying visual flickering. (*Optimize to only clear changing characters*).
- **Direct exit to Idle after Pop**: Burst test results vanish immediately on pop. (*Fix by implementing a temporary Result state*).
- **`showBoot()` Overwrite**: IDLE state immediately overwrites the boot screen. (*Refine state setup timers*).
- ~~**Duplicate include**: StateFunctions.h is included twice in StateFunctions.cpp.~~
- ~~**Serial Telemetry Rate Limits**: High 100Hz telemetry rates can overflow slow serial buffers. (*Implement lightweight binary serial packets or flow control*).~~
