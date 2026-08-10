# Wiring Guide

## Overview

The system uses a 12V rail for the Motor and a 5V rail for the Arduino/Sensors.
**Common Ground** must be connected between 12V and 5V.

## Connection Table

### Power

- **12V Source** -> Pump (+) / Voltage Divider
- **GND** -> Common System Ground

### Arduino Pins

| Pin | Connected To | Description |
| --- | --- | --- |
| **A0** | Pressure Sensor (OUT) | 0.5V - 4.5V Signal |
| **A1** | ACS712 (OUT) | Current Signal (2.5V center) |
| **A2** | Voltage Divider (OUT) | Measure 12V rail |
| **A4** | LCD (SDA) | I2C Data |
| **A5** | LCD (SCL) | I2C Clock |
| **D2** | Rotary CLK | Interrupt Pin |
| **D3** | MOSFET Gate | PWM Output (Current Limiting R required) |
| **D4** | Rotary DT | |
| **D5** | Rotary SW | Button Push |

**Note:** If using Matrix Keypad, pins D4-D11 are used (See Config.h)

### I2C Bus (A4, A5)

Multiple devices can share the I2C bus (connect in parallel):

- **LCD Display**: Address 0x27 (Default)
- **BMP280/BME280**: Address 0x76 or 0x77 (Optional Reference)

### MOSFET Driver Circuit (Low Side Switch)

1. **Source (Pin 3)** -> GND
2. **Drain (Pin 2)** -> Motor (-)
3. **Gate (Pin 1)** -> 220 Ohm Resistor -> Arduino D3
4. **Pull-Down**: 10k Resistor from **Gate** to **GND**.
5. **Flyback Diode**: Across Motor terminals (Cathode/Stripe to 12V, Anode to Motor -).

### Voltage Divider

To measure 12V safely on 5V Arduino:

- **R1 (10k)**: 12V Rail -> A2
- **R2 (2.2k)**: A2 -> GND
- Ratio: `Input = A2 * (12.2 / 2.2)`

### Sensor Power

- **Pressure Sensors**: VCC -> 5V, GND -> GND. (XGZP6847A or MPS20N0040D Module)
- **ACS712**: VCC -> 5V, GND -> GND.
- **BMP280/BME280**: VCC -> 3.3V or 5V (Check module spec), GND -> GND.
