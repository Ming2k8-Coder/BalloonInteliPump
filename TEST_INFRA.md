# E2E Test Infra: Balloon Intelligent Pump (BIP)

## Test Philosophy
- Dual-domain verification: Standalone deterministic C++ test harness for embedded physics, estimation, and safety logic, coupled with Python automated pytest/unittest runner for backend, ML, concurrency, and telemetry.
- Strict zero-failure requirement: All test cases must run programmatically and exit with code 0.
- Opaque-box and white-box coverage across Tiers 1-4:
  - Tier 1: Feature Coverage (functional validation of each individual component)
  - Tier 2: Boundary & Corner Cases (numerical stability, division by zero, negative pressures, overflow)
  - Tier 3: Cross-Feature Interactions (firmware state machine + physics + volume estimator; backend RLS + telemetry + safety cutoff)
  - Tier 4: Real-World Scenarios (complete inflation-to-burst cycle, ride dynamic cycling, long-term steady hold stability)

## Feature Inventory & Test Mapping
| # | Feature / Area | Verification Suite | Tier 1 | Tier 2 | Tier 3 | Tier 4 |
|---|----------------|-------------------|:------:|:------:|:------:|:------:|
| 1 | Mooney-Rivlin Model & Energy | C++ / Python test | 5 | 5 | ✓ | ✓ |
| 2 | Viscoelastic Creep & Fatigue | C++ / Python test | 5 | 5 | ✓ | ✓ |
| 3 | Hammerstein Volume Estimator | C++ / Python test | 5 | 5 | ✓ | ✓ |
| 4 | State Machine & Safe Transitions | C++ / Python test | 5 | 5 | ✓ | ✓ |
| 5 | RLS Pressure Predictor | Python test | 5 | 5 | ✓ | ✓ |
| 6 | Emergency Cutoffs & Burst Safety | C++ / Python test | 5 | 5 | ✓ | ✓ |
| 7 | ML Classification Pipeline | Python test | 5 | 5 | ✓ | ✓ |
| 8 | Telemetry & Concurrency | Python / C++ test | 5 | 5 | ✓ | ✓ |

## Test Architecture
- **Firmware Algorithm Test Suite**: Standalone C++ verification harness compiled with GCC/Clang or Python ctypes/native wrapper or standalone Unity harness under `test/`, exercising `bip_balloon_physics.cpp`, `bip_volume_estimator.cpp`, `bip_state.cpp` logic without hardware dependencies.
- **Backend Test Suite**: Python test suite `test_backend.py` covering `RLSPredictor` covariance stability over 50,000 samples, AR regressor validity, UDP command dispatch, mode mapping synchronization, and ML pipeline training/inference.
- **Unified Master Runner**: `python run_all_tests.py` running all test suites, verifying zero failures, and generating structured test report.

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Expected Outcome |
|---|----------|--------------------|------------------|
| 1 | Standard 11" Latex Inflation to Target | Volume Est, FSM, Mooney-Rivlin | Accurate volume & stress calculation, no divergence |
| 2 | Dynamic Ride Mode Weight-Bearing Cycling | Ride Mode, Fatigue, S-G Filter | Bounce rhythm detection, realistic fatigue accumulation |
| 3 | Predictive Burst Auto-Cutoff | RLS Predictor, Burst Detection | Pump halts when forecast exceeds 95% burst limit |
| 4 | Long-Term Pressure Hold (10-minute hold) | RLS Covariance, Volume Est | Zero covariance blow-up, zero NaN weights |
| 5 | Emergency Stop & Venting | Motor, Solenoid, Failsafe Cutoff | Immediate motor stop, full pressure venting |

## Coverage Thresholds
- Tier 1: Functional tests per module
- Tier 2: Extreme values, negative numbers, division by zero, overflow guards
- Tier 3: End-to-end integration across firmware and backend models
- Tier 4: Realistic mission profiles
