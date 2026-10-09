# Robocar ESP32 base (micro-ROS)

Owner of the ESP32 firmware and the agent link: **Alvin**. Lidar, SLAM, Nav2, camera and vision are the Pi team's. Read `INTERFACE.md` first; it is the whole contract.

**Status: not tested on a Pi.** Everything marked PLACEHOLDER is a guess until calibrated.

## Layout
```
ROBOCAR_ROS/ PlatformIO project, flashed to the ESP32-S3
  platformio.ini  custom.meta  lib/MPU6050_Custom/  src/main.cpp  src/pins.h
          
  dashboard.html      web dashboard (rosbridge, port 9090)
  autonomous_sorter.py  demo state machine (optional)
docs/                 INTERFACE.md, this README
```

## Build and flash
Open `ROBOCAR_ROS/` in VS Code with PlatformIO. After changing `custom.meta` do a clean build (the micro-ROS library is rebuilt; first build takes minutes). `custom.meta` allows 7 subscriptions and 6 publishers; the firmware needs 7 and 5. Set `upload_port` for your machine. Flash while the board is attached to the OS you build on.

## Run
```
source /opt/ros/jazzy/setup.bash
./run_all.sh
```
Open `http://<host-ip>:8000/dashboard.html`. No node after the agent starts: press the ESP32 reset button.

## Check
`ros2 node list` shows `base_controller`. `ros2 topic hz /encoders /imu_data /odometry/filtered` should show ~20 Hz (see Link budget in `INTERFACE.md`). Self-check with wheels lifted: `ros2 topic pub --once /self_check_cmd std_msgs/Int32 '{data: 1}'`, or use the dashboard button. Drive: `ros2 topic pub -r 10 /cmd_vel_raw geometry_msgs/Twist '{linear: {x: 0.2}}'`.

## Calibration (edit constants in `src/main.cpp`, reflash)
1. `TICKS_PER_REV`: wheel lifted, turn it 10 revolutions by hand, Δcount / 10.
2. Encoder direction: push the robot forward by hand; left wheels should count up, right wheels down (right is negated in firmware). Fix in firmware otherwise.
3. `WHEEL_RADIUS`: drive 2.00 m measured; scale until `/odometry/filtered` reads 2.00 m.
4. `TRACK_WIDTH` (effective): spin in place 10 turns; `track_eff = track_old × (odom yaw / true yaw)`.
5. Speed vs command: with `/max_speed_cmd` 1.0, full battery, floor, publish `/cmd_vel_raw` linear.x = 0.2 … 1.0 over 2 m and time it. The PWM floor is 150/255; add a point at the lowest moving command.
6. IMU: rotate CCW by hand, confirm `angular_velocity.z > 0`.


