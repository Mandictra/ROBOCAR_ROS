# Autonomous Waste-Sorting Mobile Manipulator (ROS 2 Jazzy)

This repository contains the firmware, control algorithms, and web dashboard for an autonomous waste-sorting Automated Guided Vehicle (AGV). The system relies on a modular architecture: a **Raspberry Pi 5** handles high-level ROS 2 autonomy (computer vision, LiDAR, path planning), while an **ESP32-S3** acts as the low-level hardware bridge via micro-ROS.

## 🛠 Hardware Architecture
*   **High-Level Brain:** Raspberry Pi 5 (Ubuntu + ROS 2 Jazzy)
*   **Low-Level Controller:** ESP32-S3 DevKit
*   **Drive System:** 4x DC Motors with hardware encoders driven by DRV8833 modules.
*   **Ejection System:** 28BYJ-48 Stepper Motor (Conveyor) + 1x Dump Servo.
*   **Perception:** TCS34725 I2C Color Sensor (Waste ID), Rear Distance Sensor (Docking), Pi Camera/LiDAR (Navigation).
*   **Camera Gimbal:** 2x Servos (Pan/Tilt).

## 💽 Software Dependencies

### ESP32-S3 (PlatformIO)
Ensure your `platformio.ini` includes the following dependencies:
```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
lib_deps =
    [https://github.com/micro-ROS/micro_ros_platformio](https://github.com/micro-ROS/micro_ros_platformio)
    madhephaestus/ESP32Encoder @ ^0.11.3
    madhephaestus/ESP32Servo @ ^3.0.5
    waspinator/AccelStepper @ ^1.64
    adafruit/Adafruit TCS34725 @ ^1.4.4
build_flags =
    -D ARDUINO_USB_CDC_ON_BOOT=0
```

### Raspberry Pi 5 / WSL Host (Ubuntu)
You must have ROS 2 Jazzy installed. Install the WebSocket bridge to enable the HTML dashboard:
```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-suite -y
```

---

## 🚀 Installation & Flashing

**1. Flash the ESP32-S3**
Compile and upload `main.cpp` using PlatformIO. This firmware utilizes the ESP32's hardware Pulse Counter (PCNT) to read the motor encoders without blocking the CPU, ensuring a stable micro-ROS connection.

**2. Set up the Python Brain**
On your Raspberry Pi (or testing laptop), save the autonomous docking and sorting logic as `autonomous_sorter.py`. Ensure it has executable permissions:
```bash
chmod +x autonomous_sorter.py
```

**3. Prepare the Dashboard**
Save `dashboard.html` to any local directory. This file uses native browser WebSockets and requires no backend server.

---

## 🏁 Running the System

To bring the entire robot online, open three separate terminals on your ROS 2 host machine (Raspberry Pi or WSL).

**Terminal 1: Start the micro-ROS Hardware Bridge**
Connect the ESP32-S3 via USB and initialize the serial agent:
```bash
source /opt/ros/jazzy/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
*(If the ESP32 is on a different port, replace `/dev/ttyUSB0` accordingly).*

**Terminal 2: Start the Web Dashboard Backend**
Launch the WebSocket server to allow the HTML file to communicate with ROS 2:
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 3: Start the Autonomous Brain**
Launch the Python node that handles color detection, docking, and the waste ejection sequence:
```bash
source /opt/ros/jazzy/setup.bash
python3 autonomous_sorter.py
```

---

## 🎮 System Usage

### Manual Tuning & Teleoperation
Double-click `dashboard.html` to open it in any web browser. 
*   **Telemetry:** Watch real-time encoder ticks and view the live color swatch reflecting the TCS34725 sensor data.
*   **Drive:** Click and hold the D-Pad to drive the robot. The Dead Man's Switch logic in the ESP32 will automatically stop the motors if the connection drops.
*   **Actuators:** Use the sliders to sweep the Pan/Tilt camera gimbal, and the buttons to test the Dump Servo and Conveyor Belt.
*   **Tuning:** Adjust the `Speed Limit` slider to dynamically update the maximum velocity parameter on the ESP32 in real-time.

### Autonomous Mode
When `autonomous_sorter.py` is running, the robot actively monitors the `/trash_color` topic. 
1. The external Pi camera (via a separate CV node) detects a bin and triggers the docking sequence.
2. The robot spins 180° and reverses until the rear distance sensor verifies it is within 5cm of the bin.
3. The rear TCS34725 color sensor confirms the bin color matches the target.
4. If a match is confirmed, the vehicle chassis tilts up to 60°, the conveyor runs to eject the waste, the chassis lowers, and the robot pulls away to resume navigation.