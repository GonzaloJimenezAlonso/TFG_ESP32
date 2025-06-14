# TFG: Prototype of a portable electronic surveillance and monitoring system

Welcome to the website and code repository for my final degree project: **Prototype of a portable electronic surveillance and monitoring system** . This project consists on the analysis of hardware and software needs for an electronic surveillance system for portable equipment.

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware Used](#hardware-used)
- [System Architecture](#system-architecture)
- [How It Works](#how-it-works)
- [Getting Started](#getting-started)
- [License](#license)

---

## Project Overview

This TFG presents a smart system for home automation and security, built around an ESP32-C3 microcontroller. The project offers two primary modes of operation: **comfort mode** (for temperature and fan control) and **anti-theft mode** (for movement and distance-based alarm triggering). The system can be controlled and monitored via a web interface, with data exchange through Bluetooth Low Energy (BLE).

---

## Features

- **Comfort Mode**
  - Real-time ambient temperature monitoring (LM35 sensor).
  - Automatic or manual control of a fan (3 speed levels).
  - LCD display for local status feedback.

- **Anti-Theft Mode**
  - Motion detection using an accelerometer.
  - Distance detection with an HC-SR04 ultrasonic sensor.
  - Audible and visual alarm (speaker and LED) on intrusion detection.
  - Alarm activation and deactivation via BLE/web interface.

- **Web Interface**
  - Switch between modes.
  - Manual fan control and display of current status.
  - Real-time notifications of temperature and alarm events.

- **BLE Communication**
  - Real-time updates and commands between ESP32-C3 and the web interface.

---

## Hardware Used

- [ESP32-C3] Microcontroller
- LM35 Temperature Sensor
- HC-SR04 Ultrasonic Distance Sensor
- ADXL335 Accelerometer (or compatible analog accelerometer)
- 16x2 I2C LCD Display
- DFRobot Digital Speaker Module (PWM)
- LED Indicator
- Assorted resistors, wiring, and breadboard or PCB
- Power supply: 5V regulated (with Schottky diode for safety)

---

## System Architecture

```
+---------------------+
|     Web Interface   | <--- BLE/HTTPS ---> | ESP32-C3 Microcontroller |
+---------------------+                      |                         |
                                             |  +------------------+   |
                                             |  |  LCD Display     |   |
                                             |  |  Fan/Motor (PWM) |   |
                                             |  |  Speaker/Buzzer  |   |
                                             |  |  LED             |   |
                                             |  |  LM35 Temp       |   |
                                             |  |  Accelerometer   |   |
                                             |  |  HC-SR04 Sonar   |   |
                                             +------------------+---+
```

---

## How It Works

### Comfort Mode
- The ESP32-C3 reads temperature from the LM35 sensor.
- Fan speed is controlled automatically based on temperature, or manually via the web interface.
- The current state is shown on the LCD and sent to the web interface via BLE.

### Anti-Theft Mode
- The ESP32-C3 monitors movement (accelerometer) and proximity (sonar).
- If significant movement or a close object is detected, the alarm is triggered.
- The LCD, LED, and speaker provide local warnings; the web interface is updated instantly.

### Power Considerations
- The system is powered through the 5V pin with protection (Schottky diode recommended).
- 3.3V peripherals are powered via the onboard regulator (max 700mA total).

---

## Getting Started

1. **Connect the hardware** as per the schematic (see `/docs/schematics/` if available).
2. **Flash the ESP32-C3** with the provided firmware (`main.ino` or equivalent).
3. **Launch the web interface** (see `/web/` or follow deployment instructions).
4. **Pair via BLE** and interact with the system!

---

## License

This project is for educational and non-commercial use as part of a university TFG.

---

**Author:** Gonzalo Jiménez Alonso  
**Contact:** [Your email or GitHub profile]
