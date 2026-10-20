# IMU guided calibration test

A four-stage guided test for the MPU6050 on the ROBOCAR_ROS robot. The terminal tells you what to do and when;
the firmware prints a set of `RESULT` lines you can paste into the commissioning record.

Full theory, procedure, acceptance criteria and troubleshooting: `imu_calibration_report.tex`.

## Files

| File | Role |
|---|---|
| `IMU_CALIBRATE.cpp` | The guided test firmware. No motors, no ROS. Flash it instead of `main.cpp`. |
| `MPU6050_Custom.cpp/.h` | Sensor driver, used unchanged. Applies the stored static offsets, so test results are residuals relative to them. |
| `attitude.h` | Roll/pitch complementary filter used by the robot firmware (not run by this test). |

## Build

Add an environment that builds only the test and the driver (the header comment in the file names it
`imu_guided_test.cpp`, which is stale):

```
[env:imu_test]
extends = env:<your existing env>
build_src_filter = -<*> +<IMU_CALIBRATE.cpp> +<MPU6050_Custom.cpp>
```

Serial monitor at 115200 baud, set to send a line ending. Any received byte counts as ENTER.

## Running

Flat, level, rigid surface; fully charged battery; a mark on the floor and one on the robot's nose; a 20 to 30 mm block.

| Stage | What you do | What it measures |
|---|---|---|
| Start-up | nothing | `WHO_AM_I`, gyro/accel config registers, `ACCEL_CONFIG2` probe (tells MPU6050 from MPU-6500 family) |
| 1. Still (8 s) | leave the robot untouched | accel means and noise, gyro bias and noise, apparent mount tilt |
| 2. Spin (30 s window) | turn it one full turn counter-clockwise by hand, smoothly, end on the mark | gyro sign and scale (reported / true) |
| 3. Nose up | lift the front about 15 to 20 degrees over 4 s, hold 3 s | accel angle vs gyro integral on pitch |
| 4. Left side up | same with the left side | same on roll |

Run the whole test three times, rebooting between runs, and copy all `RESULT` lines each time. Pressing a key after "ALL DONE" restarts the board.

## Reading the results

| Check | Pass |
|---|---|
| `WHO_AM_I` | 0x68 |
| Stage 1 sample count | about 390 to 400 |
| Accel magnitude | G +/- 0.15 m/s^2; `ax`, `ay` mean under 0.1 |
| Gyro x, y means | under 0.005 rad/s |
| Gyro std | under 0.01 rad/s (otherwise the robot moved; repeat) |
| Stage 2 verdict | CCW gives positive `gz` |
| Stage 2 scale | 0.99 to 1.01 (one turn resolves only about +/-0.7 %) |
| Stage 3/4 ratio | 0.9 to 1.1, no sign-mismatch warning |

These thresholds are practical limits chosen for this robot, not datasheet values.

## Updating constants

The printed means are residuals after the driver's stored offsets. For a channel with stored constant `c` and residual mean `m`:

- gyro x, y and accel x, y: `c_new = c + m`
- accel z: `c_new = c + (mean_az - G)`
- gyro z offset is measured at every boot by `calibrateGyro`, so it is informational only
- gyro scale: sensitivity = `131 x S_reported` LSB/(deg/s), where `S_reported` is the printed "gyro scale factor" (reported over true). Use the same constant in `calibrateGyro` and `readData`.

Only change a constant if the residual is above the limit and repeatable over three runs. Change one at a time and re-run.

For a scale error under about 0.5 % use the multi-turn test (five full turns each way, floor line, repeat twice, compare directions) instead of Stage 2.

## Known issues in this file

- Header comment names `imu_guided_test.cpp`.
- The Stage 2 verdict text mentions a `gz = -gz` line that no longer exists in `main.cpp`.
- The `ACCEL_CONFIG2` interpretation depends on the chip family; check the datasheet.
