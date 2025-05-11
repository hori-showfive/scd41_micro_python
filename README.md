# SCD41 MicroPython Driver

A MicroPython implementation for interfacing with the Sensirion SCD41 CO₂, temperature, and humidity sensor.

## Overview

This project provides a MicroPython driver for the Sensirion SCD41 sensor, enabling easy integration with MicroPython-compatible microcontrollers like ESP32. The SCD41 is a high-precision CO₂ sensor that also measures temperature and relative humidity.

## Features

- CO₂ measurement in ppm (parts per million)
- Temperature measurement in °C
- Relative humidity measurement in %RH
- I2C communication with CRC error checking
- Serial number retrieval
- Periodic measurement mode

## Hardware Requirements

- MicroPython-compatible microcontroller (e.g., ESP32)
- Sensirion SCD41 sensor
- Power supply (typically 3.3V)
- I2C connection wires

## Wiring

The default configuration uses the following pins for I2C connection:
- SCL (Serial Clock): GPIO 21
- SDA (Serial Data): GPIO 19
- I2C Bus ID: 0

You can adjust these pin assignments in the `main()` function to match your specific hardware setup.

## Usage

1. Upload the `scd41_main.py` script to your MicroPython device
2. Rename it to `main.py` if you want it to run automatically at startup
3. The script will initialize the sensor and start reading measurements every 5 seconds

### Example Code

```python
import machine
import time
from scd41_main import SCD41

# Initialize I2C
i2c = machine.I2C(0, scl=machine.Pin(21), sda=machine.Pin(19), freq=100000)

# Create SCD41 instance
scd = SCD41(i2c)

# Start periodic measurement
scd.start_periodic_measurement()

# Wait for first measurement (recommended 5+ seconds)
time.sleep(5)

# Read measurements
if scd.get_data_ready_status():
    co2, temp, humidity = scd.read_measurement()
    print(f"CO2: {co2} ppm, Temperature: {temp:.2f} °C, Humidity: {humidity:.2f} %RH")
```

## Functions

| Function | Description |
|----------|-------------|
| `start_periodic_measurement()` | Starts continuous measurement mode |
| `stop_periodic_measurement()` | Stops continuous measurement mode |
| `get_data_ready_status()` | Checks if new measurement data is available |
| `read_measurement()` | Reads CO₂, temperature, and humidity values |
| `get_serial_number()` | Retrieves the sensor's serial number |

## Error Handling

The driver includes comprehensive error handling:
- CRC validation for all sensor communications
- I2C device detection
- Exception handling for I2C communication errors

## Limitations

- The SCD41 typically has a 5-second measurement interval
- First valid measurement may require up to 5 seconds after starting periodic measurements

## Resources

- [Sensirion SCD4x Datasheet](https://sensirion.com/products/catalog/SCD41/)
- [MicroPython Documentation](https://docs.micropython.org/)

## License

This project is open source and available for any use.
