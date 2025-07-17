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
> **Note:**  
> Prior to this project, I conducted an analysis of the system requirements using the Texas Instruments F28335 microcontroller.  
> You can find the analysis code here: [`F28335_analysis/main.c`](https://github.com/GonzaloJimenezAlonso/TFG_ESP32/blob/main/TFG_short.c).

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
### PIN LAYOUT
![image](https://github.com/user-attachments/assets/a4d01df6-7396-41a5-aa3c-14b76e6b2393)

Below is the pin assignment for connecting the ESP32-C3 to external components. Pin numbers refer to the ESP32-C3’s GPIOs.

| ESP32-C3 Pin | Connected Hardware                                                                       | Notes/Links                                                                                          |
|--------------|-----------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------|
| GPIO2        | [LM35 Temperature Sensor](https://wiki.dfrobot.com/DFRobot_LM35_Linear_Temperature_Sensor__SKU_DFR0023_) (Analog OUT)                 | Reads analog temperature (A0)                                                                        |
| GPIO3        | [ADXL335 Accelerometer](https://wiki.dfrobot.com/Triple_Axis_Accelerometer_MMA7361_SKU_DFR0143) (X axis Analog OUT) | X-axis analog output                                                                                |
| GPIO4        | [ADXL335 Accelerometer](https://wiki.dfrobot.com/Triple_Axis_Accelerometer_MMA7361_SKU_DFR0143) (Y axis Analog OUT) | Y-axis analog output                                                                                |                                                                             |
| GPIO4        | [HC-SR04 Ultrasonic Sensor](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf) (Trigger)   | Trigger pin                                                                                         |
| GPIO5        | [HC-SR04 Ultrasonic Sensor](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf) (Echo)      | Echo pin                                                                                            |
| GPIO6        | [16x2 I2C LCD Display](https://learn.adafruit.com/i2c-spi-lcd-backpack/overview) (SDA)                | SDA line for I2C                                                                                   |
| GPIO7        | [16x2 I2C LCD Display](https://learn.adafruit.com/i2c-spi-lcd-backpack/overview) (SCL)                | SCL line for I2C                                                                                   |
| GPIO8        | [Buzzer/Speaker](https://www.electronicwings.com/nodemcu/buzzer-interfacing-with-nodemcu) (PWM OUT)    | Alarm sound                                                                                         |
| GPIO9        | [LED Indicator](https://www.sparkfun.com/products/9590) (Digital OUT)                                   | Alarm indicator                                                                                     |
| GPIO10       | [Fan/Motor Module](https://components101.com/motors/5v-dc-fan) (PWM OUT)                                | Fan control (PWM)                                                                                   |
| 3V3          | Power for sensors (as needed)                                                                            | Up to 700mA from onboard regulator                                                                  |
| 5V           | Main power input/output                                                                                  |(https://www.vishay.com/docs/88540/ss14.pdf) for safety                  |
| GND          | Common ground for all components                                                                        |                                                                                                     |

> **Note:**  
> - Double-check your ESP32-C3 board’s silkscreen or documentation for exact pin header locations.
> - Some I2C LCD modules might use different default I2C addresses (usually 0x27 or 0x3F).

---

### **Wiring Diagram**

```
LM35 (OUT) -------- GPIO2 (ADC)
Accelerometee (X/Y) --- GPIO3/4 (ADC)
HC-SR04 (Trig/Echo) -- GPIO10/9
LCD (SDA/SCL) ----- GPIO6/7 (I2C)
Buzzer ----------- GPIO21 (PWM)
LED -------------- GPIO8 (Digital)
Fan -------------- GPIO20 (PWM)
All VCCs --------- 5V
All GNDs --------- GND
```

---

**For more details, see each component’s datasheet linked above.**

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
2. **Flash the ESP32-C3** with the provided firmware (see [`main.ino`](https://github.com/GonzaloJimenezAlonso/TFG_ESP32/blob/main/main.ino)).
3. **Launch the web interface**  You can access directly at: [https://gonzalojimenezalonso.github.io/TFG_ESP32/](https://gonzalojimenezalonso.github.io/TFG_ESP32/)
4. **Pair via BLE** and interact with the system!

---

## License

This project is for educational and non-commercial use as part of a university TFG.

---

**Author:** Gonzalo Jiménez Alonso  
**Contact:** [gonjimalo@alum.us.es](mailto:gonjimalo@alum.us.es)
