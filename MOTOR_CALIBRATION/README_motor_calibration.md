# Motor calibration and PID autotune

Autotunes the four wheel speed loops of the ROBOCAR_ROS robot (ESP32-S3, 4 encoder DC motors).
A Python dashboard talks over serial to a small test firmware, identifies each motor, computes PI gains and
feedforward, verifies them in closed loop and writes them to `tuned_gains.json` and `gains.h`.

Full theory, derivations and figures: `motor_calibration_report.tex`.

## Files

| File | Role |
|---|---|
| `PID_TEST.cpp` | Test firmware. 50 ms control loop, per-motor PID with separate forward and reverse gain sets, open-loop PWM mode, serial command interface. Flash it instead of the robot firmware. |
| `motor_calibrate.py` | Tk and matplotlib dashboard: live plot, manual sliders, autotune, step test, speed sweep, save, flash. |
| `autotune_math.py` | Pure maths, no hardware: plant identification (`identify`), SIMC PI tuning (`simc_pi`), shared closed-loop speed (`common_tau_c`), step metrics (`step_metrics`). |

## Requirements

- Python 3 with `pyserial`, `matplotlib`, `python-dotenv`, and Tk (`tkinter`).
- PlatformIO (`pio`) if you want the dashboard to flash firmware.
- `pins.h` in the firmware project (pin definitions and `MOTOR_DIR_*`).
- Encoders: 748 counts per wheel revolution (half-quadrature). One count in 50 ms is 60 / (748 x 0.05) = 1.6 rpm.

## Setup

1. Put the wheels in the air. The autotune drives the motors open loop at PWM 150, 180, 210 and more.
2. Flash `PID_TEST.cpp` (the dashboard's **UPLOAD PID TEST** button does it if `ROBOT_PROJECT` is set).
3. Optional `.env` next to `motor_calibrate.py`:

   ```
   SERIAL_PORT=/dev/ttyUSB0
   BAUD_RATE=115200
   ROBOT_PROJECT=/home/you/robot     # gains.h is written to <ROBOT_PROJECT>/include/gains.h
   PIO_ENV_ROBOT=robot
   PIO_ENV_TEST=pid_test
   ```
4. `python motor_calibrate.py`

`SAMPLE_DT` in the script must equal `SAMPLE_TIME_MS` in the firmware (both 0.05 s / 50 ms).

## Using it

- **AUTOTUNE (selected / all):** open-loop steps at 150, 180, 210 PWM, then extra steps aimed at 15 and 35 rpm. Fits gain K, dead-band PWM, lag tau and dead time L per motor, computes SIMC PI gains with a shared closed-loop time constant, then verifies at about 30 rpm. If overshoot exceeds 5 % or ripple exceeds 3 rpm, Kp and Ki are multiplied by 0.75 (up to 3 rounds). Feedforward is then tried at 0.4, 0.7, 1.0 of 1/K and the fastest acceptable one is kept.
- **STEP TEST** and **SPEED SWEEP** (10 to 120 rpm, forward and reverse): check results. The sweep flags speeds where the motors differ by more than 2.5 rpm.
- **SAVE GAINS:** writes `tuned_gains.json` and the `gains.h` header. **SAVE + UPLOAD ROBOT FIRMWARE** also builds and flashes the robot environment.
- **STOP MOTORS** sends zero targets. **RESET TO DEFAULT** restores the dashboard defaults without touching the saved file.

## Serial protocol (`PID_TEST.cpp`)

Line format `<cmd><sel><value>\n`, `sel` = `A` (all) or `1`..`4` (FL, FR, RL, RR).

| cmd | Meaning |
|---|---|
| `T` | Target rpm, closed loop, clamped to +/-120. Any `T` leaves open-loop mode. |
| `P` `I` `D` | Kp, Ki, Kd (forward) |
| `F` `B` | Feedforward PWM per rpm, feedforward offset PWM (forward) |
| `p` `i` `d` `f` `b` | The same five gains for negative targets (reverse) |
| `M` | Raw open-loop PWM, -255 to 255 |

Example: `TA30` sets all motors to 30 rpm. Output every 50 ms: `Target:30 FL:.. FR:.. RL:.. RR:..`.

## Notes and caveats

- Kd mostly amplifies the 1.6 rpm quantisation, so the autotune leaves it at 0.
- Results are only as good as the open-loop identification: use fully charged batteries (a "wrong sign" result earlier turned out to be dead batteries).
- The simulation figures in the report come from a synthetic plant and are illustrative, not measurements of your motors.
- The script constants (test levels, tau_c factors, limits) are at the top of `motor_calibrate.py`.
