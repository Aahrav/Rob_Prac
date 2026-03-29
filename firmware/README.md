# Firmware for ESP32 + MPU-6050

## Components
- ESP32 board
- MPU-6050 IMU sensor

## Wiring
Connect MPU-6050 to ESP32:
- VCC → 3.3V
- GND → GND
- SDA → GPIO21
- SCL → GPIO22

## Setup
1. Install Arduino IDE
2. Add ESP32 board manager URL: https://dl.espressif.com/dl/package_esp32_index.json
3. Install "ESP32" board via Arduino Boards Manager
4. Install MPU6050 library via Library Manager (Search "MPU6050" by Electronic Cats)
5. Select your ESP32 board and port
6. Open `mpu6050_test.ino` and upload

## Expected Output
Serial Monitor at 115200 baud should show raw accelerometer and gyroscope values:
```
ax: 0.12 ay: 0.34 az: 9.81 gx: 0.01 gy: -0.02 gz: 0.00
...
```
