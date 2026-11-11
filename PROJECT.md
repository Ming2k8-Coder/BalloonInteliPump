# Project: Balloon Intelligent Pump (BIP) Full-Stack Audit & Remediation

## Architecture
The Balloon Intelligent Pump (BIP) is a closed-loop pneumatic control and telemetry system composed of three primary tiers:
1. **ESP32 Native Firmware (`main/`, `components/`, `test/`)**:
   - Dual-core FreeRTOS architecture: Core 1 runs high-frequency real-time sensing (`control_task` at 2000 SPS) via TI ADS1220 ADC and LEDC PWM pump/solenoid valve actuation. Core 0 handles network communication (`net_task` UDP telemetry, `httpd` REST/SSE server, `uart_task` command interpreter).
   - Embedded physics & estimation engine: Equibiaxial Mooney-Rivlin hyperelastic stress model, Standard Linear Solid (SLS) viscoelastic creep, Miner's rule fatigue accumulation, Hammerstein pneumatic flow volume estimator, Savitzky-Golay digital filter, and 11-mode finite state machine.
2. **Python Backend & Analytics Service (`server/`)**:
   - Telemetry processing engine with UDP/Serial ingestion and Server-Sent Events (SSE) streaming.
   - Recursive Least Squares (RLS) adaptive pressure forecasting model for predictive burst prevention.
   - Scikit-learn machine learning classification pipeline (`train_ml.py`) for automated balloon state identification.
3. **Desktop & Web GUI Clients (`gui/`, `server/app.py:HTML_GUI`)**:
   - Real-time strip-charts, calibration controls, manual and automated inflation profiles.

## Feature Inventory
| # | Feature / Defect | Description | Milestone | Source |
|---|------------------|-------------|-----------|--------|
| 1 | FW-PIN-01 | Fix GPIO 3 pin conflict (LEDC PWM colliding with UART0 RX) | M1 | Survey 1 |
| 2 | FW-SENS-01 | Fix ADS1220 driver Reg 1 config (TS=0, 2000 SPS Turbo mode) & SPI CS framing | M1 | Survey 1 |
| 3 | FW-CONC-01 | Cross-core synchronization for currentMode, sensors, physics, and buffers | M1 | Survey 1 |
| 4 | FW-SAFE-01 | Fix motor overcurrent debounce false-trip (1.5ms to 25ms inrush tolerance) | M1 | Survey 1 |
| 5 | FW-SAFE-02 | Global real-time overpressure safety cutoff in control_task | M1 | Survey 1, 3 |
| 6 | FW-SAFE-03 | Connect burst_shock_detected flag to motor shutdown and valve venting | M1 | Survey 3 |
| 7 | FW-SAFE-04 | Emergency STOP command failsafe venting (release trapped pressure) | M1 | Survey 3 |
| 8 | FW-PERF-01 | Optimize or offload Pop Black Box 4000-sample processing from Core 1 | M1 | Survey 1, 3 |
| 9 | ALG-MR-01 | Replace uniaxial invariants with equibiaxial spherical membrane equations | M2 | Survey 3 |
| 10 | ALG-MR-02 | Correct strain energy integration to use rubber volume instead of air volume | M2 | Survey 3 |
| 11 | ALG-MR-03 | Fix Miner's rule fatigue stress units (MPa scaling vs 100% destruction on cycle 1) | M2 | Survey 3 |
| 12 | ALG-RLS-01 | Fix online Mooney-Rivlin RLS parameter identification sample count divergence | M2 | Survey 1, 3 |
| 13 | ALG-FSM-01 | Invoke add_sample() in all active modes so dP/dt and d2P/dt2 are computed | M2 | Survey 1 |
| 14 | ALG-VOL-01 | Add relief valve deflation flow model to volume estimator (prevent volume drift) | M2 | Survey 3 |
| 15 | ALG-VOL-02 | Fix diameter control mode actuator latch-up and hoop stress pressure formula | M2 | Survey 3 |
| 16 | ALG-FSM-02 | Allocate Event Group bit for MODE_CONDITION and fix SMART_FINISHED deadlock | M2 | Survey 3 |
| 17 | PY-CRASH-01 | Fix Windows cp1252 UnicodeEncodeError crash on startup in app.py & train_ml.py | M3 | Survey 2 |
| 18 | PY-RLS-01 | Fix RLS target sample data leakage in update() (prevent identity passthrough) | M3 | Survey 2, 3 |
| 19 | PY-RLS-02 | Fix RLS covariance matrix windup, divide-by-zero, and explosive forecast bounds | M3 | Survey 2, 3 |
| 20 | PY-NET-01 | Resolve UDP command/telemetry connection handshake and deadlock | M3 | Survey 2 |
| 21 | PY-SYNC-01 | Synchronize mode code definitions (0-10) across backend, GUI, and firmware | M3 | Survey 2 |
| 22 | PY-SAFE-01 | Extend predictive burst cutoff to all active inflating modes (1-5, 8-10) | M3 | Survey 2, 3 |
| 23 | PY-ML-01 | Fix train_ml.py classification report crash on missing classes and schema mismatch | M3 | Survey 2 |
| 24 | PY-THRD-01 | Fix thread safety for concurrent deque iteration and CSV logging | M3 | Survey 2 |
| 25 | TEST-E2E-01 | Standalone C++ unit test suite for firmware algorithms (MR, SLS, Vol, FSM) | M4 | Survey 1, 2, 3 |
| 26 | TEST-E2E-02 | Automated Python backend test suite (RLS stability, ML pipeline, socket/API) | M4 | Survey 2 |
| 27 | TEST-E2E-03 | End-to-end automated verification runner asserting 100% zero-failure pass rate | M4 | Survey 1, 2, 3 |
| 28 | DOC-AUDIT-01 | Consolidated technical audit report docs/AUDIT_REPORT.md per R5 | M5 | Survey 1, 2, 3 |

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| M1 | Firmware Safety, Concurrency & Drivers Remediation | Items 1-8: GPIO remap, ADS1220 fix, atomic/mutex sync, OC debounce, pressure cutoffs, burst failsafe | none | DONE (Verified: test/verify_m1_remediation.py 8/8 PASS) |
| M2 | Mathematical Models, Algorithms & FSM Remediation | Items 9-16: Mooney-Rivlin equibiaxial, strain energy, Miner's rule, online RLS, add_sample, volume deflation | M1 | DONE (Verified: test/verify_m2_remediation.py 9/9 PASS, stress_test_m2.py 6/6 PASS) |
| M3 | Backend, Streaming & ML Pipeline Remediation | Items 17-24: Windows Unicode fix, RLS data leakage & windup fix, UDP handshake, mode sync, ML pipeline | none | DONE (Verified: test/verify_m3_remediation.py 8/8 PASS, stress_test_m3 19/19 PASS, forensic CLEAN) |
| M4 | Automated Verification Harness & E2E Test Suite | Items 25-27: C++ algorithm unit tests, Python backend tests, automated test runner | M1, M2, M3 | DONE (Verified: run_all_tests.py 12/12 PASS, 100% zero-failure rate, TEST_READY.md published) |
| M5 | Comprehensive Audit Synthesis & Report Generation | Item 28: Consolidated docs/AUDIT_REPORT.md | M4 | DONE (Verified: docs/AUDIT_REPORT.md 1135 lines, 86 KB, R1-R5 verified) |

## Interface Contracts
### Firmware Telemetry Packet (`bip_config.h` & `server/app.py` / `gui/app.py`)
- Packet structure: 15 fields, semicolon or comma-separated CSV string.
- Mode codes: 0=IDLE, 1=MANUAL, 2=SMART, 3=BURST, 4=PULSE, 5=PATTERN, 6=CALIBRATION, 7=ERROR, 8=DIAMETER, 9=RIDE, 10=CONDITION.
- Commands: `CONNECT`, `STOP`, `MODE <id>`, `PWM <val>`, `VENT <val>`, `TARGET <val>`, `TARE`, `PING`.
- Predictive burst cutoff: Triggered when `forecasted_p >= estimated_burst * 0.95` across all inflating modes (1, 2, 3, 4, 5, 8, 9, 10).

### Volume Estimator ↔ State Machine
- `update_volume_estimator_ext(p_kpa, pwm_motor, pwm_solenoid, temp_c, dt)`
- Solenoid venting at rate proportional to pressure differential and valve opening.

### Mooney-Rivlin Physics ↔ State Machine
- `update_balloon_physics_ext(p_kpa, dp_dt, d2p_dt2, temp_c, dt)`
- Cauchy wall stress: $\sigma = 2(\lambda^2 - \lambda^{-4})(C_{10} + \lambda^2 C_{01})$
- Strain energy: $E_{\text{strain}} = W_{\text{density}} \times V_{\text{rubber}}$ where $V_{\text{rubber}} = 4\pi r_0^2 t_0$.

## Code Layout
- `main/`: ESP32 firmware C++ source files
- `test/`: C++ unit testing files (Unity framework / standalone harness)
- `server/`: Python backend, RLS forecasting model, ML training pipeline, web GUI
- `gui/`: Tkinter desktop GUI
- `docs/`: Technical documentation and consolidated `AUDIT_REPORT.md`
- `.agents/teamwork/`: Orchestrator and agent metadata
