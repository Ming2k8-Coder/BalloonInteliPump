# Balloon Testing & Data Acquisition Best Practices

This document outlines the engineering best practices for setting up the Balloon Intelligent Pump (BIP) hardware, optimizing high-speed sensor data, and physically mounting balloons to achieve accurate, scientific-grade burst (B2P) results.

---

## 1. High-Speed Data Acquisition (ADS1220 ADC)

To achieve the 100Hz+ telemetry goal with high precision (24-bit), the ADS1220 must be optimized to reduce noise and maximize signal integrity.

### Hardware & PCB Layout

* **Avoid Breadboards:** Parasitic capacitance and loose wires introduce massive noise. Use a custom PCB or a high-quality shielded breakout board.
* **Ground Planes & Power:** Keep analog and digital signals strictly separated. Decouple the analog supply (AVDD) from the digital supply (DVDD) using ferrite beads and bypass capacitors (0.1µF and 1µF) placed close to the chip.
* **Anti-Aliasing:** Place an RC low-pass filter immediately before the analog input pins (AIN0-AIN3) to block RF noise.

### Configuration & Software

* **Data Rate (DR):** Faster rates equal more noise. For 100Hz telemetry, set the ADS1220 to **175 SPS (Turbo Mode)** or **90 SPS (Normal Mode)**. Do not use the maximum 2000 SPS unless strictly required, as it reduces the signal-to-noise ratio.
* **Interrupt-Driven:** Do not poll the SPI bus continuously. Connect the `DRDY` (Data Ready) pin to a microcontroller interrupt to trigger SPI reads, minimizing digital bus noise.
* **Software Filtering:** Implement a fast Exponential Moving Average (EMA) filter on the microcontroller to smooth out remaining jitter.

### Secondary Measurements (MUX)

The ADS1220 has 4 analog channels, allowing simultaneous measurements alongside pressure:

* **Internal:** Read ambient room temperature using the chip's internal sensor.
* **Temperature Probes:** Use the built-in IDACs to drive a PT100/PT1000 probe inside the balloon to track thermodynamic changes (adiabatic heating/cooling).
* **Other Sensors:** Connect analog mass airflow sensors or linear displacement potentiometers to unused channels (AIN1-AIN3).

---

## 2. Capturing the "Pop" (Burst Dynamics)

* **Using the ADS1220 (2000 SPS):** At maximum speed (1 sample every 0.5ms), the ADC is perfect for capturing the absolute peak burst pressure right before failure.
* **Using an Oscilloscope:** To capture the exact mechanical shockwave and millisecond-level pressure drop, connect an oscilloscope directly to the analog output of the XGZP6847A sensor. Set a **"Falling Edge" trigger** slightly below the max expected voltage to automatically freeze the frame during the burst.

---

## 3. Physical Strain Measurement

Traditional metallic foil strain gauges cannot be glued directly to latex; they create rigid "hard spots" that cause premature popping.

### Recommended Methods

1. **The Hoop Method (Indirect):** Wrap a non-stretchy string or specific elastic band around the equator of the uninflated balloon. Attach the string to a standard load cell mounted on the rig frame. As the balloon expands, it pulls the string, measuring circumferential force.
2. **Soft Stretch Sensors:** Glue highly conductive, flexible rubber/silicone sensors directly to the balloon's equator using silicone-based adhesive.
3. **Digital Image Correlation (DIC):** The professional standard. Draw a dot grid on the balloon and use a camera + software (like OpenCV) to track the displacement of the dots during inflation.

---

## 4. Balloon Orientation for Burst (B2P) Tests

Gravity deforms latex. Incorrect mounting concentrates stress at the neck, ruining the test.

* **Standard Round Balloons (9" - 16"):**
  * *Orientation:* **Neck Down (Vertical)**.
  * The nozzle points straight up. The balloon rests on the nozzle and expands symmetrically.
* **Very Long "Twister" Balloons (160, 260, 350):**
  * *Orientation:* **Horizontal**.
  * Lay the balloon flat on a long, smooth table or inside a half-cut PVC pipe so the inflation bubble can travel down the tube without kinking.
* **Giant Balloons (24" - 72"):**
  * *Orientation:* **Neck Down (Supported)**.
  * See "The Support Plate" below.

---

## 5. Preventing Body-Neck Transition Pops

The zone where the thick neck flares into the thin body is highly vulnerable. To ensure a true membrane burst at the equator, you must protect the transition zone.

### Engineering Fixes

1. **Clamp Low:** Insert the nozzle only 1-2 cm into the neck, near the rolled lip. Do not push the pipe up into the transition zone, which prevents it from expanding freely.
2. **Smooth Nozzle:** The tip of the air pipe must be perfectly chamfered and polished. A sharp cut pipe will slice the latex as it stretches.
3. **Conditioning:** Pre-stretch the balloon manually, or use `MODE_CONDITIONING` to inflate to 30% and deflate before the final test. This warms the latex and aligns the polymer chains.
4. **The Support Plate (For Giant Balloons):** Heavy balloons will pull their own bodies down, ripping the neck on the nozzle. Route the air pipe through a smooth, flat plate (like a plastic disc). The heavy balloon body rests on the plate, taking all the gravitational strain off the neck transition zone while keeping the airway open.
