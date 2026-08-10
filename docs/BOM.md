# Bill of Materials (BOM)

## Core Components

| Item | Description | Quantity | Note |
| --- | --- | --- | --- |
| **Microcontroller** | Arduino Nano (V3) or Uno | 1 | |
| **Pump** | 12V High-Pressure Diaphragm Pump | 1 | "555-4000", "ES3910", or similar |
| **Pressure Sensor** | XGZP6847A (0-200kPa) | 1 | Default |
| **Pressure Sensor** | MPS20N0040D-S (0-40kPa) | 1 | Alternative Option |
| **Ref Sensor** | BMP280 or BME280 | 1 | Optional (For Auto-Cal) |
| **Current Sensor** | ACS712-05B (5A Range prefered) | 1 | Analog Output |
| **MOSFET** | IRLZ44N (Logic Level) | 1 | To drive pump |
| **Diode** | HER508 (Ultrafast) or 1N5408 | 1 | Flyback protection |
| **LCD** | 16x2 LCD with I2C Backpack | 1 | Address 0x27 |
| **Power Supply** | Any type | 1 | 5V for arduino |

## Interface

| Item | Description | Quantity | Note |
| --- | --- | --- | --- |
| **Input** | Rotary Encoder (KY-040) | 1 | Recommended |
| **OR** | 4x4 Matrix Keypad | 1 | Alternative |

## Passive Components

| Item | Value | Quantity | Location |
| --- | --- | --- | --- |
| **Resistor** | 220 Ohm | 1 | MOSFET Gate |
| **Resistor** | 10k Ohm | 1 | MOSFET Pull-down |
| **Resistor** | 10k Ohm | 1 | Voltage Divider R1 |
| **Resistor** | 2.2k Ohm | 1 | Voltage Divider R2 |
| **Capacitor** | 100nF (104) | 3 | Motor EMI Filter |

## Misc

- Silicone Tubing (ID to match pump/sensor).
- T-Connector (Barbed).
- Breadboard or PCB.
- Jumper Wires.
