# AGV Line Follower Robot

An autonomous line-following robot built with ESP32-S3, using LDR sensor array and PID control algorithm. Features a real-time web dashboard accessible via WiFi.

## Hardware

### Sensor Board
- 8x LDR (photoresistor) sensors
- 16x LED 5mm (IR emitters)
- 8x 220Ω resistors
- 10kΩ pull-down resistor array

### Control Board
- ESP32-S3 (main MCU)
- LM2596 5V buck converter (step-down from 12V)
- L298N motor driver
- HC-SR04 ultrasonic sensor (obstacle detection)
- Battery voltage monitor (10kΩ / 3.3kΩ divider)
- 3x LED indicators (Green / Yellow / Red)

## Features

- **Auto mode:** PID-based line following with 8 LDR sensors
- **Manual mode:** Remote control via web interface D-pad
- **Obstacle detection:** Auto-stop when object detected within 20cm
- **Battery monitor:** Real-time voltage display with LED warning
- **Web dashboard:** Hosted on ESP32 WiFi AP (no router needed)
- **Real-time data:** WebSocket updates every 250ms

## PID Algorithm

```cpp
Kp = 2.200204
Ki = 0.000001  
Kd = 4.170504
baseSpeed = 200
```

Error is calculated from 8-sensor bitmask pattern.
PID output adjusts left/right motor speed differentially.

## Tech Stack

| Component | Details |
|---|---|
| MCU | ESP32-S3-DevKitC-1-N8R8 |
| Framework | Arduino (ESP-IDF) |
| Language | C/C++ |
| Web | HTML + CSS + JavaScript (WebSocket) |
| PCB Design | EasyEDA |
| Power | 12V LiPo → LM2596 → 5V |

## How to Connect

1. Power on the robot
2. Connect to WiFi: **AGV_BINH** / Password: **12345678**
3. Open browser → go to **192.168.4.1**
4. Dashboard loads automatically

## Project Structure

## Academic Info

- **Project type:** Graduation Thesis
- **University:** Hanoi University of Industry (HAUI)
- **Major:** Electronics & Telecommunications
- **Score:** 8.5 / 10
