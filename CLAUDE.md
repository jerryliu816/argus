# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This is a Raspberry Pi hardware interface codebase called "bytebeast" that provides libraries and examples for various HAT (Hardware Attached on Top) modules:

- **Display Module**: 0.96" and 1.3" LCD displays with SPI interface
- **Sense HAT**: Multi-sensor board with IMU, pressure, humidity, temperature sensors
- **Environment Sensors**: BME280, ICM20948, LTR390, SGP40, TSL2591 sensor modules
- **UPS HAT**: Uninterruptible Power Supply monitoring with INA219

## Code Architecture

### Core Components
1. **display/**: LCD display drivers and examples
   - `lib/`: Core display libraries (LCD_0inch96.py, LCD_1inch3.py, lcdconfig.py)
   - `example/`: Test scripts for different display configurations
   - Supports both SPI0 and SPI1 interfaces

2. **Sense_HAT_C_Pi/**: Comprehensive sensor HAT with multiple language support
   - `RaspberryPi/`: Contains modules for IMU, LPS22HBTR, SGM58031, SHTC3, TCS34087
   - Each sensor module has implementations in C (bcm2835, lgpio, wiringPi) and Python

3. **environment/**: Environmental sensor modules (BME280.py, ICM20948.py, etc.)

4. **UPS_HAT_C/**: Power management with INA219.py for current/voltage monitoring

5. **lib/**: Contains third-party libraries and demos
   - WiringPi library for GPIO control
   - bcm2835 library for low-level access
   - lg-master (lgpio/rgpio) for modern GPIO access

### Hardware Interface Libraries
The project supports multiple GPIO access methods:
- **lgpio/rgpio**: Modern GPIO library (recommended)
- **wiringPi**: Traditional GPIO library
- **bcm2835**: Low-level hardware access
- **Python smbus/spidev**: Python hardware interfaces

## Development Commands

### Display Testing
```bash
cd display/example
sudo python3 0inch96_spi0ce0.py    # Test first 0.96" screen
sudo python3 0inch96_spi0ce1.py    # Test second 0.96" screen
sudo python3 1inch3_spi1ce0.py     # Test 1.3" screen (requires SPI1 enabled)
sudo python3 double_0inch96_spi.py # Test dual displays
sudo python3 CPU.py                # Display CPU information
sudo python3 key_double.py         # Test buttons
```

### Environment Sensor Testing
```bash
cd environment/
python3 BME280.py     # Test temperature/humidity/pressure
python3 ICM20948.py   # Test IMU sensor
python3 LTR390.py     # Test UV/light sensor
python3 SGP40.py      # Test air quality sensor
python3 TSL2591.py    # Test light sensor
python3 test.py       # Run all sensor tests
```

### Sense HAT Testing
```bash
cd Sense_HAT_C_Pi/RaspberryPi/[sensor]/python/
python3 [sensor_file].py   # Test individual sensors
```

### Building C Libraries (if needed)
```bash
cd lib/demos/lg-master/
make                  # Build lgpio/rgpio libraries
```

## Hardware Configuration
- Enable SPI: Add `dtparam=spi=on` to /boot/config.txt
- Enable SPI1 (for 1.3" display): Add `dtoverlay=spi1-1cs` to /boot/config.txt
- Enable I2C: Add `dtparam=i2c_arm=on` to /boot/config.txt

## Dependencies
- Python libraries: smbus, spidev, PIL (Pillow), numpy
- Run with sudo for hardware access
- Raspberry Pi OS with GPIO libraries enabled

## Development Workflow
1. First think through the problem, read the codebase for relevant files, and write a plan to todo.md
2. The plan should have a list of todo items that you can check off as you complete them
3. Before you begin working, check in with me and I will verify the plan
4. Then, begin working on the todo items, marking them as complete as you go
5. Please every step of the way just give me a high level explanation of what changes you made
6. Make every task and code change you do as simple as possible. We want to avoid making any massive or complex changes. Every change should impact as little code as possible. Everything is about simplicity
7. Finally, add a review section to the todo.md file with a summary of the changes you made and any other relevant information

## File Patterns
- Python sensor modules follow pattern: `SensorName.py` with class `SensorName`
- C implementations have separate directories for each GPIO library (bcm2835, lgpio, wiringPi)
- Display examples use format: `[size]_[interface].py`
- Configuration files use `lcdconfig.py` or `DEV_Config.c/h` patterns