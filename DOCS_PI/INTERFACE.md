# ESP32 base interface (v0.4)

This file is the contract between the ESP32 firmware (Alvin) and the Pi/laptop side (lidar, SLAM, Nav2, camera, vision). The firmware may change; this file is what you can rely on. Anything marked PLACEHOLDER is a guess until calibrated.

**Node:** `base_controller` (ESP32-S3, micro-ROS, serial **115200 baud** through a UART bridge, agent on the Pi/laptop).
**Conventions:** REP-103 (x forward, y left, z up), SI units, yaw positive = counter-clockwise, everywhere including the firmware heading hold.
**QoS:** every ESP32 publisher is **best-effort**. Subscribe with best-effort / `qos_profile_sensor_data`, otherwise you receive nothing.

## Frames
`map → odom → base_link → imu_link, laser_link, camera_link`. `base_link` is the rotation centre between the wheels and must be the only URDF root.
The ESP32 publishes the odometry *message* only, never TF. **The Pi side publishes the TF `odom → base_link`** from `/odometry/filtered`. URDF, lidar/camera frames and all sensor offsets belong to the Pi team.

## ESP32 publishes
| Topic | Type | Rate | Content |
|---|---|---|---|
| `/encoders` | `std_msgs/Int32MultiArray` | 20 Hz | `[FL, RL, FR, RR, stamp_sec, stamp_nsec]`, raw half-quadrature counts (int32, may wrap). Right-side counts count *down* when driving forward (firmware negates them internally). |
| `/imu_data` | `sensor_msgs/Imu` | 20 Hz | frame `imu_link`; MPU6050, ±2 g, ±250 °/s, ~44 Hz DLPF. `angular_velocity.z` is bias-corrected rad/s. `orientation` is **yaw only** (x = y = 0), taken from the EKF; orientation covariance is not set. |
| `/odometry/filtered` | `nav_msgs/Odometry` | 20 Hz | `odom → base_link`. Onboard EKF: gyro-z drives the prediction, encoder-derived yaw corrects it. Pose covariance from the EKF; twist covariance is a PLACEHOLDER. Twist is in `base_link`. |
| `/trash_color` | `geometry_msgs/Vector3` | 10 Hz | raw TCS34725 R, G, B (16-bit counts) in x, y, z. |
| `/self_check` | `std_msgs/Int32` | 1 Hz | self-check result bitfield, 2 bits per item, bit 30 toggles when a new result is ready. Started by `/self_check_cmd`. |

Exactly one publisher may exist on `/odometry/filtered`. Do not run a second filter (e.g. `robot_localization`) that publishes the same topic name, and do not fuse `/imu_data` gyro into another filter together with `/odometry/filtered`: the gyro is already inside it.

Timestamps come from the micro-ROS agent time sync. If the agent failed to sync, stamps are far from the Pi clock: fall back to arrival time (a >1 s difference is a good test).

## ESP32 subscribes
| Topic | Type | Behaviour |
|---|---|---|
| `/cmd_vel_raw` | `geometry_msgs/Twist` | **Normalised, not m/s.** `linear.x`, `angular.z` ∈ [−1, 1]. Firmware mixes `left = lin − w`, `right = lin + w`, then scales by `/max_speed_cmd`. **Motors stop 500 ms after the last command**: publish at ≥ 2 Hz (10 Hz recommended). |
| `/max_speed_cmd` | `std_msgs/Float32` | Scale 0.1–1.0 on `/cmd_vel_raw`. Publish 1.0 when you do your own speed mapping. |
| `/heading_hold_cmd` | `std_msgs/Bool` | `false` disables the firmware heading hold (required while Nav2 drives). Default is `true` on boot **and after every agent reconnect**, so republish `false` at ~1 Hz. |
| `/pan_tilt_cmd` | `geometry_msgs/Point` | `x` pan 0–180°, `y` tilt 30–150° (clamped). |
| `/dump_cmd` | `std_msgs/Float32` | Dump servo angle 5–175°. |
| `/conveyor_cmd` | `std_msgs/Float32` | Conveyor stepper speed, steps/s, max 700. |
| `/self_check_cmd` | `std_msgs/Int32` | Any value starts a ~7 s self-check. **The robot moves its motors and servos; wheels off the ground.** The main loop is blocked during it. |

## What the Pi side must provide (not part of this repo)
1. TF `odom → base_link` from `/odometry/filtered`.
2. `/cmd_vel` (m/s, rad/s, what Nav2 publishes) → `/cmd_vel_raw` (normalised): measure speed against command on the floor and build the mapping. The ESP32 speed is **open loop**; it varies with battery.
3. `/max_speed_cmd` = 1.0 and `/heading_hold_cmd` = `false` while Nav2 drives.
4. micro-ROS agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev <port> -b 115200`, on the same ROS 2 distro as the firmware (`board_microros_distro`, currently `jazzy`). The serial device is the UART bridge (CH340/CP210x), **not** the S3 native USB. Opening the port may reset the board; the firmware reconnects by itself.
5. Everything else: lidar, SLAM, Nav2, camera, vision.

## Link budget (important)
115200 baud is about 11.5 kB/s. By estimate, `/odometry/filtered` is ~720 B and `/imu_data` ~320 B per message, so both at 20 Hz need about 21 kB/s: more than the link carries. Best-effort messages are then dropped silently. Verify with `ros2 topic hz`; the fix is a higher baud (changed in firmware `Serial.begin` and the agent `-b`) or lower rates. This file will state the final baud once decided.

## Known limitations
- Yaw drifts: gyro integration corrected only by encoder yaw, which is wrong under skid-steer slip. Absolute heading must come from SLAM/AMCL.
- Odometry constants (`WHEEL_RADIUS` 0.0325 m, `TRACK_WIDTH` 0.20 m, `TICKS_PER_REV` 1000) are PLACEHOLDERS compiled into the firmware; recalibrating means reflashing.
- IMU is mounted upside down; firmware flips only `gz`. Axes x/y and accel are not corrected: do not trust them for orientation.
