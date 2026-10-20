# ROBOCAR_ROS: Autonomous Waste-Sorting Robot (ESP32-S3 + micro-ROS + ROS 2 Jazzy)

A four-wheel skid-steer robot that drives, senses and sorts waste. An **ESP32-S3** runs the real-time work (wheel speed control, encoders, IMU, colour sensor, servos, conveyor, state estimation, failsafe) and joins a **ROS 2 Jazzy** graph through **micro-ROS** over a serial cable. A host computer (laptop under WSL 2, or a Raspberry Pi on the robot) runs the micro-ROS agent, `twist_mux`, rosbridge and the navigation stack. A browser **dashboard** provides manual control and live telemetry.

> **Full documentation:** [`docs/ROS2_ESP32_Setup_Guide-v3.tex`](docs/ROS2_ESP32_Setup_Guide-v3.tex) is the complete manual (setup from scratch, how micro-ROS works, firmware internals, dashboard, troubleshooting). This README is the short version.

---

## Contents

- [Features](#features)
- [System overview](#system-overview)
- [Repository layout](#repository-layout)
- [Hardware](#hardware)
- [Quick start](#quick-start)
- [Daily use](#daily-use)
- [ROS 2 interface](#ros-2-interface)
- [Web dashboard](#web-dashboard)
- [Configuration and tuning](#configuration-and-tuning)
- [Safety](#safety)
- [Troubleshooting](#troubleshooting)
- [Known issues](#known-issues)
- [Building the manual](#building-the-manual)
- [Author and license](#author-and-license)

---

## Features

- micro-ROS client on the ESP32-S3 (node `base_controller`), serial transport at 460 800 baud, with automatic reconnection when the agent restarts or the cable is replugged.
- Per-wheel PI speed control with feed-forward and separate forward/reverse gains.
- IMU attitude filter and a 4-state EKF publishing `odometry/filtered` for Nav2.
- Optional heading hold for straight-line driving.
- Command timeout failsafe (500 ms) that works even when the agent is lost.
- On-demand self-check (IMU, colour sensor, four motors, servos, stepper).
- Web dashboard with drive pad, keyboard, Xbox controller, actuator controls and live telemetry.

## System overview

```
Browser (dashboard.html) --WebSocket:9090--> rosbridge --\
                                                          +--> ROS 2 Jazzy (twist_mux, Nav2, SLAM)
                                       micro-ROS agent <--/
                                              |
                                 USB serial, 460800 baud
                                              |
                                    ESP32-S3 (micro-ROS client)
                       motors, encoders, servos, stepper, MPU6050, TCS34725
```

## Repository layout

```
.
├── platformio.ini            Build settings for the ESP32-S3
├── custom.meta               micro-ROS entity limits (read when the micro-ROS library is built)
├── src/
│   ├── main.cpp              Firmware: micro-ROS, control loops, estimation, self-check
│   └── MPU6050_Custom.cpp    IMU driver
├── include/
│   ├── pins.h                Pin map, motor polarity, limits, command timeout
│   ├── gains.h               Per-wheel PI gains (AUTO-GENERATED, do not edit by hand)
│   ├── attitude.h            Roll/pitch complementary filter
│   └── MPU6050_Custom.h      IMU driver header
├── tuned_gains.json          Calibration output that gains.h is generated from
├── twist_mux_config.yaml     Manual (priority 50) vs Nav2 (priority 10) command sources
├── run_all.sh                Starts agent, rosbridge, twist_mux and the web server
├── dashboard.html            Web dashboard
└── docs/
    └── ROS2_ESP32_Setup_Guide-v3.tex   Full manual
```

`gains.h` is produced from `tuned_gains.json` by the calibration tooling (`motor_calibrate.py`, named in the header of `gains.h`). That tooling is not included in this repository yet.

## Hardware

| Part | Notes |
|---|---|
| ESP32-S3 dev board | `esp32-s3-devkitc-1`, 16 MB flash, octal PSRAM; two USB ports (UART bridge and native USB) |
| 4 x DC gear motors with H-bridge drivers | Half-quadrature encoders, 748 counts per wheel turn; wheel radius 38.1 mm |
| MPU6050 (clone) | I2C `0x68`, `WHO_AM_I` reads `0x72` |
| TCS34725 colour sensor | I2C `0x29` |
| 2 servos | Tilt and dump |
| 28BYJ-48 stepper | Conveyor |
| Host | Windows 11 laptop with WSL 2 (development) or Raspberry Pi with Ubuntu Server 24.04 (robot) |

All pin numbers live in [`include/pins.h`](include/pins.h). GPIO46 (`ENC_FL_A`) is a strapping pin on the ESP32-S3.

## Quick start

These are the short steps. The manual's Chapter 2 gives every command with explanations and checks.

### 1. Host setup (Ubuntu 24.04, or WSL 2 on Windows)

```bash
sudo apt update && sudo apt install -y git curl cmake build-essential python3-pip python3-venv
```

- **Windows only:** install WSL 2 (`wsl --install -d Ubuntu-24.04` in an administrator PowerShell) and `usbipd-win` (`winget install --interactive --exact dorssel.usbipd-win`). Share the board with WSL:
  ```powershell
  usbipd list
  usbipd bind --busid <BUSID>
  usbipd attach --wsl --busid <BUSID>     # repeat after every re-plug
  ```
  Use the board's **UART** USB port (appears as `/dev/ttyUSB0`), not the native USB port.
- Allow serial access, then restart WSL / log in again:
  ```bash
  sudo usermod -a -G dialout $USER
  ```
- Install **ROS 2 Jazzy** following the [official instructions](https://docs.ros.org/en/jazzy/Installation.html) (`ros-jazzy-ros-base`, `ros-dev-tools`, `python3-colcon-common-extensions`), and add `source /opt/ros/jazzy/setup.bash` to `~/.bashrc`.
- Install the other ROS packages:
  ```bash
  sudo apt install -y ros-jazzy-rosbridge-suite ros-jazzy-twist-mux
  ```

### 2. micro-ROS agent

Try the package first (if it exists for your setup), otherwise build from source:

```bash
sudo apt install -y ros-jazzy-micro-ros-agent        # option A, if available
```

```bash
# option B: build from source
sudo rosdep init && rosdep update
mkdir -p ~/microros_ws/src && cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
cd ~/microros_ws
rosdep install --from-paths src --ignore-src -y
colcon build && source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```

If built from source, also uncomment the `source ~/microros_ws/install/setup.bash` line in `run_all.sh`.

### 3. PlatformIO

On Ubuntu 24.04 the `apt` version of `pio` is broken. Use the official installer:

```bash
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
echo 'export PATH=$PATH:$HOME/.platformio/penv/bin' >> ~/.bashrc && source ~/.bashrc
pio --version
```

### 4. Clone and flash

```bash
git clone https://github.com/<your-username>/<repo-name>.git ROBOCAR_ROS
cd ROBOCAR_ROS
chmod +x run_all.sh
pio run                 # first build compiles the micro-ROS library (several minutes)
pio run -t upload       # agent must NOT be running
```

Check `platformio.ini` matches your board (flash size, PSRAM) and that `upload_port` is your serial port.

> **Do not use `Serial.print` or the PlatformIO serial monitor.** `Serial` carries the micro-ROS data stream; printing to it breaks the connection. Debug with topics or an LED.

### 5. Run

Place the robot **still and level** (the IMU calibrates for about 3 s at boot) and, for the first test, **on blocks with the wheels off the ground**. Then:

```bash
./run_all.sh /dev/ttyUSB0 460800
```

In another terminal:

```bash
ros2 node list                                   # /base_controller
ros2 topic hz /odometry/filtered                 # ~20 Hz
ros2 topic echo /encoders --qos-reliability best_effort --once
```

Open `http://localhost:8000/dashboard.html`.

## Daily use

1. Attach the board to WSL (Windows only): `usbipd attach --wsl --busid <BUSID>`.
2. Robot still and level, powered on.
3. `./run_all.sh /dev/ttyUSB0 460800` (the ESP32 may boot before or after it and will reconnect).
4. Open the dashboard; the connection badge turns green.
5. **Before flashing, stop `run_all.sh`.** The agent holds the serial port.

`run_all.sh` arguments: `[PORT] [BAUD]`, defaults `/dev/ttyUSB0` and `460800`. The baud rate **must** equal `MICROROS_BAUD` in `main.cpp`. Ctrl+C stops everything.

**Phone access (WSL 2):** WSL's NAT hides ports 8000 and 9090. Enable mirrored networking (`networkingMode=mirrored` under `[wsl2]` in `%UserProfile%\.wslconfig`, then `wsl --shutdown`), allow the ports in Windows Firewall, and open `http://<laptop-wifi-ip>:8000/dashboard.html`. See the manual (Error 13) for details.

**Raspberry Pi:** install ROS 2 and the agent as above, plug the ESP32 straight in (no `usbipd`), and give the ESP32 and lidar fixed device names with udev rules so the agent never opens the wrong port. The manual's Section "Moving to the Raspberry Pi" has the steps.

## ROS 2 interface

Node `base_controller`. Sensor topics are **best-effort**; any subscriber must request best-effort QoS, or it receives nothing.

**Published by the ESP32**

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `encoders` | `std_msgs/Int32MultiArray` | 20 Hz | `[FL, RL, FR, RR, stamp_sec, stamp_nsec]` |
| `imu_data` | `sensor_msgs/Imu` | 20 Hz | frame `imu_link` |
| `odometry/filtered` | `nav_msgs/Odometry` | 20 Hz | `odom` to `base_link`, EKF output |
| `trash_color` | `geometry_msgs/Vector3` | 10 Hz | raw R, G, B |
| `self_check` | `std_msgs/Int32` | 1 Hz | 2 bits per item; bit 30 toggles per completed run |

**Subscribed by the ESP32**

| Topic | Type | Meaning |
|---|---|---|
| `cmd_vel_raw` | `geometry_msgs/Twist` | m/s and rad/s; `angular.x > 0.5` requests heading hold |
| `pan_tilt_cmd` | `geometry_msgs/Point` | `y` = tilt in degrees (30 to 150); `x` ignored |
| `dump_cmd` | `std_msgs/Float32` | dump servo angle (clamped 15 to 165) |
| `conveyor_cmd` | `std_msgs/Float32` | stepper speed, steps/s (limit 700) |
| `self_check_cmd` | `std_msgs/Int32` | any value starts the self-check (wheels will spin) |
| `heading_hold_cmd` | `std_msgs/Bool` | master enable for heading hold |
| `max_speed_cmd` | `std_msgs/Float32` | obsolete, ignored |

**Velocity path:** dashboard (`cmd_vel_teleop`, priority 50) and Nav2 (`cmd_vel_nav`, priority 10) go into `twist_mux`, whose output is remapped to `cmd_vel_raw`. Each source times out after 0.5 s.

The ESP32 has fixed-size micro-ROS tables set in `custom.meta`: 1 node, 6 publishers, 7 subscriptions. **All 7 subscriptions are in use.** To add one, raise `RMW_UXRCE_MAX_SUBSCRIPTIONS` and rebuild the micro-ROS library.

## Web dashboard

Served by `run_all.sh` on port 8000.

- **Drive:** on-screen pad, W/A/S/D or arrow keys, Space to stop, speed-limit slider.
- **Actuators:** camera tilt, dump/reset, conveyor speed.
- **Telemetry:** heading, encoders, acceleration, roll/pitch, colour swatch, self-check results.
- **Xbox controller** (standard mapping, page focused): D-pad drives, triggers run the conveyor, right stick moves the camera, A/X/Y dump, reset bin, centre camera, RB/LB change the speed limit.
- URL options: `?host=<ip>` connects to rosbridge on another machine; `?debug` shows outgoing commands without a robot.
- `roslib` and `eventemitter2` are loaded from a CDN, so the viewing device needs internet unless you serve local copies.

## Configuration and tuning

| To change | Edit |
|---|---|
| Pin or motor polarity | `include/pins.h` (`MOTOR_DIR_*`) |
| Wheel speed response | Re-run calibration, regenerate `gains.h` from `tuned_gains.json` |
| Turning behaviour | `TRACK_CMD`, `MAX_YAW_RATE` in `main.cpp` |
| Straight-line holding | `HOLD_KP`, `HOLD_KD`, `HOLD_MAX` in `main.cpp` |
| Serial speed | `MICROROS_BAUD` in `main.cpp` **and** the agent's `-b` |
| IMU/odometry publish rate | `IMU_ODOM_PUB_DIV` in `main.cpp` |
| Command timeout | `CMD_TIMEOUT_MS` in `include/pins.h` |
| micro-ROS entity limits | `custom.meta`, then `pio run -t clean_microros` and rebuild |

At 115 200 baud the robot's data (about 21.8 kB/s by calculation) exceeds the link capacity (11.5 kB/s). Use 460 800, or raise `IMU_ODOM_PUB_DIV`.

## Safety

- Run the first tests and the **self-check with the wheels off the ground**. The self-check spins every wheel.
- The robot stops 500 ms after the last velocity command, including when the agent is lost.
- The dashboard has no emergency-stop button. Use Space, the Stop button, or release the control. Stopping does not stop the conveyor.
- rosbridge has no authentication: anyone on the same network can publish to the robot. Use a trusted network.

## Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| Upload fails, port busy or missing | Agent is running, or the board is not attached to WSL. Stop `run_all.sh`; run `usbipd attach` again. |
| `/dev/ttyUSB0` missing | Board not attached, charge-only cable, or wrong USB port. |
| Permission denied on the port | Add yourself to `dialout`, then restart WSL / log in. |
| Agent runs but no node | Baud rates differ, or `ARDUINO_USB_CDC_ON_BOOT=0` is missing from `build_flags`. |
| Connects, then drops at random | Link saturated or baud mismatch. Use 460 800. |
| `/self_check` or `/heading_hold_cmd` missing | Subscription limit reached. Raise it in `custom.meta`, rebuild the library. |
| Telemetry empty but drive works | QoS mismatch. Subscribe best-effort; check `ros2 topic info <topic> -v`. |
| "Extrapolation" / "message too old" | ESP32 clock sync failed. Restart the agent. |
| Dashboard unreachable from a phone | WSL NAT. Use mirrored networking and open the firewall ports. |
| Self-check names shifted on the dashboard | Dashboard list has a "Pan servo" entry the firmware lacks (see below). |

The manual's Chapter 6 has 19 entries with causes and fixes.

## Known issues

These were found while documenting the code and are not yet fixed:

1. `dashboard.html` self-check list has 10 items including "Pan servo"; the firmware reports 9. Actuator results appear one row too low.
2. Motor self-check codes 0 and 3 carry diagnostic meaning in the firmware but are labelled "Not run" / "Ran, check by eye" on the dashboard.
3. No emergency-stop button, although the styling for one exists.
4. The pan slider controls a servo that no longer exists; tilt and dump slider ranges differ from the firmware's clamps.
5. The dashboard heading hint says "gyro + encoders"; the EKF heading is gyro-only.
6. `MPU6050_Custom.cpp` comments a gyro-z scale factor (0.793) that the code does not apply.
7. Heading-hold gains were tuned before the command moved to rad/s and need re-tuning.
8. Comments in `main.cpp` mention 921 600 baud and a 0.20 m track; the code uses 460 800 and 0.215 m.

## Building the manual

```bash
cd docs
latexmk -pdf ROS2_ESP32_Setup_Guide-v3.tex
```

Requires a TeX Live installation with `lmodern`, `microtype`, `tikz`, `booktabs`, `tabularx`, `listings` and `cleveref`.

## Author and license

Alvin Sam Omondi Gamba.

Add a `LICENSE` file before publishing (for example MIT or Apache-2.0); none has been chosen yet.