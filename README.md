# ATMOS22 Logger

A BLE-enabled firmware that reads weather data from an ATMOS22 sonic anemometer and broadcasts it to connected peripheral devices via Bluetooth Low Energy.

## Overview

This firmware runs on a RAKwireless WisBlock device with an attached RAK13010 SDI-12 interface module. It acts as a **BLE central hub**, reading wind speed, direction, and temperature from the ATMOS22 sensor and transmitting this data to up to 4 connected BLE peripherals. It's designed as a companion to [ZEPHIRuS](https://github.com/toneja/ZEPHIRuS), which runs on the peripheral nodes that receive this weather data.

## Features

- Real-time weather measurements from ATMOS22 sonic anemometer
- BLE central mode supporting up to 4 simultaneous peripheral connections
- 128×64 OLED display showing wind speed, direction, and temperature
- 1 Hz sampling rate (configurable)
- Status LEDs for sensor polling and BLE activity
- Debug output via Serial

## Hardware

- **Microcontroller**: RAKwireless WisBlock Core with BLE support
- **Sensor Interface**: RAK13010 SDI-12 interface module
- **Sensor**: METER ATMOS22 sonic anemometer
- **Display**: SSD1306 128×64 OLED (I2C)
