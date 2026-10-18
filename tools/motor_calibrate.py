import collections
import json
import os
import shutil
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog

import matplotlib.pyplot as plt
import serial
from dotenv import load_dotenv
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from autotune_math import identify, simc_pi, step_metrics, common_tau_c

load_dotenv()

# --- CONFIGURATION ---
SERIAL_PORT = os.getenv('SERIAL_PORT', '/dev/ttyUSB0')
BAUD_RATE = int(os.getenv('BAUD_RATE', 115200))
GAINS_FILE = os.getenv('GAINS_FILE', 'tuned_gains.json')
# Robot firmware project (PlatformIO). If set, saved gains are written to <ROBOT_PROJECT>/include/gains.h and the
# "SAVE + UPLOAD ROBOT" button builds and flashes it. All of these can go in a .env file next to this script.
ROBOT_PROJECT = os.getenv('ROBOT_PROJECT', '')                  # e.g. /home/you/robot
PIO_CMD = os.getenv('PIO_CMD', shutil.which('pio') or os.path.expanduser('~/.platformio/penv/bin/pio'))  # auto-detected
UPLOAD_PORT = os.getenv('UPLOAD_PORT', '')                      # port used for uploading (empty = same as SERIAL_PORT)
PIO_ENV_ROBOT = os.getenv('PIO_ENV_ROBOT', 'robot')             # platformio.ini environments: the robot firmware ...
PIO_ENV_TEST = os.getenv('PIO_ENV_TEST', 'pid_test')            # ... and the PID test firmware this dashboard talks to
# C++ header written by SAVE GAINS / autotune. Empty string = do not write it.
GAINS_HEADER = os.getenv('GAINS_HEADER', os.path.join(ROBOT_PROJECT, 'include', 'gains.h') if ROBOT_PROJECT else 'gains.h')
AUTOLOAD_GAINS = os.getenv('AUTOLOAD_GAINS', '1') == '1'   # 0 = start with DEFAULT_GAINS instead of the saved file
# Gains applied by RESET TO DEFAULT (the dashboard's original start-up values). Edit if your firmware defaults differ.
DEFAULT_GAINS = {"P": 2.0, "I": 5.0, "D": 0.0, "F": 3.4, "B": 0.0}

SAMPLE_DT = 0.05          # must match SAMPLE_TIME_MS in firmware
TEST_PWMS = (150, 180, 210)  # open-loop levels; all must clearly move the motor. Not clamped to 70 rpm: lift the wheels
TEST_DURATION = 2.5       # s per step, should be several time constants
SETTLE_STOP = 1.5         # s to coast to rest between steps
TAU_C_FACTOR = 0.25      # closed-loop speed: lower = more aggressive (tau_c >= TAU_C_FACTOR*tau)
TAU_C_L_MULT = 1.25      # tau_c >= TAU_C_L_MULT*L ; 1.0 is the SIMC limit, 2.0 was the old robust setting
FF_SCALE = 0.0            # starting feedforward gain as a fraction of 1/K. 0 = deadband offset only
FF_SPEEDUP = (0.4, 0.7, 1.0)  # after the PI is accepted, try these FF fractions; keep the fastest that stays within limits
ADAPTIVE_LEVELS = True    # after the first fit, add open-loop steps at the PWM predicted to give REFINE_RPMS (covers low speeds)
REFINE_RPMS = (15.0, 35.0)  # wheel speeds the extra identification steps aim for
MIN_TEST_PWM = 40         # never test below this PWM (inside the dead-band the motor does not move)
SWEEP_SPEEDS = (10.0, 20.0, 30.0, 50.0, 70.0, 100.0, 120.0)  # firmware clamps at 120  # speeds of the SPEED SWEEP (forward and reverse); firmware clamps at 70
SWEEP_SPREAD_RPM = 2.5    # sweep flags a speed when the motors' final rpm differ by more than this
SWEEP_AFTER_AUTOTUNE = True  # run the sweep automatically after a successful autotune
VERIFY_RPM = 30.0         # closed-loop verification step after tuning (capped at 80% of the fast open-loop step)
RIPPLE_MAX_RPM = 3.0      # steady-state peak-to-peak limit; 2 counts (2.4 rpm) is quantization hunting, only >2 counts is real oscillation
MAX_OVERSHOOT_PCT = 5.0  # if the verification overshoots more, gains are detuned and re-checked
DETUNE_FACTOR = 0.75      # Kp and Ki are multiplied by this per detune round
MAX_DETUNE_ROUNDS = 3

MOTOR_KEYS = {"1": "FL", "2": "FR", "3": "RL", "4": "RR"}
GAIN_KEYS = ("P", "I", "D", "F", "B")  # Kp, Ki, Kd, feedforward PWM/rpm, feedforward offset PWM (forward, target > 0)
REV_KEYS = ("Pr", "Ir", "Dr", "Fr", "Br")  # same five gains used when the target is negative (firmware commands p,i,d,f,b)
ALL_GAIN_KEYS = GAIN_KEYS + REV_KEYS
AUTOTUNE_REVERSE = True   # also identify/tune/verify every motor in reverse (negative PWM, negative rpm target)

max_points = 100
data = {k: collections.deque(maxlen=max_points) for k in ('Target', 'FL', 'FR', 'RL', 'RR')}

# Per-motor memory. "A" only holds the target; gains always live per motor.
motor_params = {m: {"P": 2.0, "I": 5.0, "D": 0.0, "F": 3.4, "B": 0.0,
                    "Pr": 2.0, "Ir": 5.0, "Dr": 0.0, "Fr": 3.4, "Br": 0.0, "T": 0} for m in MOTOR_KEYS}
motor_params["A"] = {"T": 0}

updating_gui = False
autotuning = False
abort_autotune = False
refresh_needed = False
status_text = "Ready. Autotune spins motors up to PWM %d: lift the wheels." % max(TEST_PWMS)
capture = None  # list while an autotune step is being recorded
is_paused = False

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()


def set_status(text):
    global status_text
    status_text = text


def gain_cmd(key):
    """Firmware command letter: upper case = forward set, lower case = reverse set ('Pr' -> 'p')."""
    return key if len(key) == 1 else key[0].lower()


def push_gains(m):
    for key in ALL_GAIN_KEYS:
        ser.write(f"{gain_cmd(key)}{m}{motor_params[m][key]}\n".encode('utf-8'))
        time.sleep(0.01)  # keep under the ESP32 RX buffer


def write_gains_header(path):
    """Write the gains as a C++ header: constexpr MotorGains GAINS_FL/FR/RL/RR = {{forward}, {reverse}}."""
    names = {"1": "FL", "2": "FR", "3": "RL", "4": "RR"}
    lines = ["// AUTO-GENERATED by motor_calibrate.py - do not edit by hand.",
             "#pragma once", "",
             "struct Gain { float kp, ki, kd, ff, off; };   // PWM/rpm, PWM/(rpm*s), -, PWM/rpm, PWM",
             "struct MotorGains { Gain fwd, rev; };          // fwd: target > 0, rev: target < 0", ""]
    for m, name in names.items():
        p = motor_params[m]
        fwd = ", ".join(f"{p[k]}f" for k in GAIN_KEYS)
        rev = ", ".join(f"{p[k + 'r']}f" for k in GAIN_KEYS)
        lines.append(f"constexpr MotorGains GAINS_{name} = {{{{{fwd}}}, {{{rev}}}}};")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def save_gains():
    out = {m: {k: motor_params[m][k] for k in ALL_GAIN_KEYS} for m in MOTOR_KEYS}
    if os.path.exists(GAINS_FILE):
        shutil.copy(GAINS_FILE, GAINS_FILE + ".bak")   # never lose the previous set
    with open(GAINS_FILE, 'w') as f:
        json.dump(out, f, indent=2)
    msg = f"Gains saved to {GAINS_FILE}"
    if GAINS_HEADER:
        try:
            write_gains_header(GAINS_HEADER)
            msg += f" and {GAINS_HEADER}"
        except Exception as e:
            msg += f" (gains.h NOT written: {e})"
    set_status(msg)


def load_gains(path=None):
    """Load gains from a JSON file ({"1": {"P":..,"I":..,"D":..,"F":..,"B":..}, ...}) into motor_params.
    Returns the number of values loaded (0 if the file is missing or has no matching keys)."""
    path = path or GAINS_FILE
    if not os.path.exists(path):
        return 0
    with open(path) as f:
        saved = json.load(f)
    n = 0
    for m in MOTOR_KEYS:
        for k in ALL_GAIN_KEYS:
            if k in saved.get(m, {}):
                motor_params[m][k] = float(saved[m][k])
                n += 1
        for k in GAIN_KEYS:                       # old files have no reverse set: reverse = forward
            if k + "r" not in saved.get(m, {}) and k in saved.get(m, {}):
                motor_params[m][k + "r"] = motor_params[m][k]
    return n


# Boards that reset on port open need a moment before they listen.
time.sleep(2)
if AUTOLOAD_GAINS:
    load_gains()
for _m in MOTOR_KEYS:
    push_gains(_m)


def toggle_pause():
    global is_paused
    is_paused = not is_paused
    pause_btn.config(text="▶ RESUME PLOT" if is_paused else "⏸ PAUSE PLOT")


def sync_sliders():
    """Snap the sliders to the stored values for the selected motor, without sending anything."""
    global updating_gui
    updating_gui = True
    target = target_var.get()
    gain_sliders = (p_slider, i_slider, d_slider)
    for s in gain_sliders:
        s.config(state=tk.NORMAL)  # Tk ignores set() on a disabled Scale
    rpm_slider.set(motor_params[target]["T"])
    if target != "A":
        suf = "r" if dir_var.get() == "R" else ""
        p_slider.set(motor_params[target]["P" + suf])
        i_slider.set(motor_params[target]["I" + suf])
        d_slider.set(motor_params[target]["D" + suf])
    for s in gain_sliders:
        s.config(state=tk.DISABLED if target == "A" else tk.NORMAL)
    # I think Scale.set() fires its command from an idle callback, i.e. after this function would
    # return. Flush it now while updating_gui is still True so the snap never reaches the ESP32.
    root.update_idletasks()
    updating_gui = False


def on_motor_select():
    sync_sliders()


def send_command(prefix, value):
    """Store the value for the selected motor and send it. 'All' only broadcasts the target RPM;
    gains are never overwritten from 'All', each motor keeps its own tuned set."""
    if updating_gui or autotuning:
        return
    target = target_var.get()
    if target == "A" and prefix != "T":
        return
    key, cmd = prefix, prefix
    if prefix in ("P", "I", "D") and dir_var.get() == "R":   # sliders edit the reverse set
        key, cmd = prefix + "r", prefix.lower()
    motor_params[target][key] = float(value)
    ser.write(f"{cmd}{target}{value}\n".encode('utf-8'))


def stop_all_motors():
    global updating_gui, abort_autotune
    abort_autotune = True
    updating_gui = True
    for key in motor_params:
        motor_params[key]["T"] = 0
    rpm_slider.set(0)
    root.update_idletasks()
    updating_gui = False
    try:
        ser.write(b"TA0\n")
    except Exception:
        pass


# ---------------- AUTOTUNE ----------------
def wait_stopped(m):
    time.sleep(SETTLE_STOP)
    key = MOTOR_KEYS[m]
    if data[key] and abs(data[key][-1]) > 2.0:
        time.sleep(1.5)


def capture_cmd(m, cmd):
    """Send one command to motor m (which must be at rest) and record its RPM for TEST_DURATION."""
    global capture
    key = MOTOR_KEYS[m]
    capture = []
    ser.write(cmd.encode('utf-8'))
    time.sleep(TEST_DURATION)
    cap, capture = capture, None
    ser.write(f"T{m}0\n".encode('utf-8'))  # back to closed loop at 0 -> output 0
    rpm = [s[key] for s in cap]
    if len(rpm) < 0.8 * TEST_DURATION / SAMPLE_DT:
        raise RuntimeError(f"only {len(rpm)} samples in {TEST_DURATION}s: serial data lost or SAMPLE_DT wrong")
    return rpm


def capture_step(m, pwm):
    """Open-loop PWM step on motor m from rest. Returns RPM samples at SAMPLE_DT."""
    return capture_cmd(m, f"M{m}{pwm}\n")


def capture_closed_loop_step(m, rpm_target):
    """Closed-loop target step 0 -> rpm_target on motor m with its current gains."""
    return capture_cmd(m, f"T{m}{rpm_target:.1f}\n")


def _suf(d):
    return "" if d == "F" else "r"


def identify_motor(m, d):
    """Open-loop steps forward (d='F') or reverse (d='R', negative PWM). Stage 1: TEST_PWMS. Stage 2 (ADAPTIVE_LEVELS):
    extra steps at the PWM predicted to give REFINE_RPMS, so the model also fits the LOW speeds.
    Reverse data is sign-flipped so identify() always works on positive magnitudes. None if aborted."""
    name, sign, tag = MOTOR_KEYS[m], (1 if d == "F" else -1), ("fwd" if d == "F" else "rev")
    levels, rpms = [], []

    def run(pwm, note):
        set_status(f"Autotune {name} {tag}: open-loop step {sign * pwm} PWM{note}")
        rpms.append([sign * x for x in capture_step(m, sign * pwm)])
        levels.append(pwm)
        wait_stopped(m)
        return not abort_autotune

    for pwm in TEST_PWMS:
        if not run(pwm, ""):
            return None
    res = identify(levels, rpms, SAMPLE_DT)
    if ADAPTIVE_LEVELS:
        for target in REFINE_RPMS:
            pwm = int(round(min(250.0, max(float(MIN_TEST_PWM), res["pwm0"] + target / res["K"]))))
            if any(abs(pwm - l) < 8 for l in levels):
                continue
            if not run(pwm, f" (aiming at {target:.0f} rpm)"):
                return None
        res = identify(levels, rpms, SAMPLE_DT)
    print(f"{name} {tag}: K={res['K']:.3f} rpm/PWM, deadband={res['pwm0']:.1f} PWM, tau={res['tau']:.3f}s, "
          f"L={res['L']:.3f}s (fit rms {res['fit_rms']:.2f} rpm, linearity err {res['lin_err_rpm']:.2f} rpm, "
          f"levels {sorted(levels)})")
    return res


def _verify_step(m, d, verify_rpm, label):
    name, sign, tag, suf = MOTOR_KEYS[m], (1 if d == "F" else -1), ("fwd" if d == "F" else "rev"), _suf(d)
    p = motor_params[m]
    push_gains(m)
    set_status(f"Autotune {name} {tag}: verifying at {sign * verify_rpm:.0f} rpm ({label})")
    rpm = [sign * x for x in capture_closed_loop_step(m, sign * verify_rpm)]
    wait_stopped(m)
    met = step_metrics(rpm, verify_rpm, SAMPLE_DT)
    rise = f"{met['rise90_s']:.2f}s" if met["rise90_s"] is not None else "never"
    print(f"{name} {tag}: verify {sign * verify_rpm:.0f} rpm [{label}] -> overshoot {met['overshoot_pct']:.1f}%, rise90 {rise}, "
          f"settle {met['settle_s']:.2f}s, ripple {met['ripple_pp']:.1f} rpm pp, final {sign * met['final_rpm']:.1f} rpm "
          f"(Kp={p['P' + suf]} Ki={p['I' + suf]} F={p['F' + suf]})")
    met["ok"] = met["overshoot_pct"] <= MAX_OVERSHOOT_PCT and met["ripple_pp"] <= RIPPLE_MAX_RPM
    return met


def verify_motor(m, d, res):
    """Closed-loop step in direction d. Phase 1: detune PI on overshoot/ripple.
    Phase 2: raise the feedforward fraction step by step and keep the fastest setting within limits."""
    p, suf = motor_params[m], _suf(d)
    verify_rpm = max(10.0, min(VERIFY_RPM, 0.8 * res["ss_max"]))
    met = None
    for rnd in range(MAX_DETUNE_ROUNDS + 1):
        met = _verify_step(m, d, verify_rpm, f"PI round {rnd + 1}")
        if abort_autotune:
            return
        if met["ok"] or rnd == MAX_DETUNE_ROUNDS:
            break
        p["P" + suf] = round(p["P" + suf] * DETUNE_FACTOR, 2)
        p["I" + suf] = round(p["I" + suf] * DETUNE_FACTOR, 2)
    if met["overshoot_pct"] > MAX_OVERSHOOT_PCT or met["rise90_s"] is None:
        return
    best_F, best_rise = p["F" + suf], met["rise90_s"]
    for ff in FF_SPEEDUP:
        p["F" + suf] = round(ff / res["K"], 3)
        met = _verify_step(m, d, verify_rpm, f"FF {ff:.1f}/K")
        if abort_autotune:
            return
        if met["ok"] and met["rise90_s"] is not None and met["rise90_s"] < best_rise - 0.02:
            best_F, best_rise = p["F" + suf], met["rise90_s"]
        else:
            break
    p["F" + suf] = best_F
    push_gains(m)
    print(f"{MOTOR_KEYS[m]} {'fwd' if d == 'F' else 'rev'}: final F={best_F}, rise90 {best_rise:.2f}s")


def autotune_worker(selection):
    global autotuning, refresh_needed
    autotuning = True
    try:
        motors = list(MOTOR_KEYS) if selection == "A" else [selection]
        dirs = ("F", "R") if AUTOTUNE_REVERSE else ("F",)
        ser.write(b"TA0\n")
        time.sleep(0.5)

        plants = {}
        for m in motors:                      # pass 1: identify every motor in every direction
            for d in dirs:
                res = identify_motor(m, d)
                if res is None:
                    break
                plants[(m, d)] = res
            if abort_autotune:
                break
        if abort_autotune or len(plants) < len(motors) * len(dirs):
            set_status("Autotune aborted")
            return

        tau_c = common_tau_c(plants.values(), TAU_C_FACTOR, TAU_C_L_MULT)   # pass 2: one shared tau_c, all motors/directions
        print(f"shared tau_c = {tau_c:.3f}s")
        for (m, d), res in plants.items():
            kp, ki = simc_pi(res["K"], res["tau"], res["L"], tau_c=tau_c)
            suf = _suf(d)
            motor_params[m].update({"P" + suf: round(kp, 2), "I" + suf: round(ki, 2), "D" + suf: 0.0,
                                    "F" + suf: round(FF_SCALE / res["K"], 3), "B" + suf: round(res["pwm0"], 1)})
        if not AUTOTUNE_REVERSE:                # keep the reverse set equal to forward
            for m in motors:
                for k in GAIN_KEYS:
                    motor_params[m][k + "r"] = motor_params[m][k]

        for (m, d), res in plants.items():    # pass 3: verify / detune
            verify_motor(m, d, res)
            if abort_autotune:
                set_status("Autotune aborted")
                return
        save_gains()
        if SWEEP_AFTER_AUTOTUNE:
            n = speed_sweep()
            set_status("Autotune done, gains saved; speed sweep OK" if n == 0
                       else f"Autotune done, gains saved; speed sweep flagged {n} speed(s), see the console")
        else:
            set_status("Autotune done, gains saved and applied")
    except Exception as e:
        ser.write(b"TA0\n")
        set_status(f"Autotune failed: {e}")
    finally:
        autotuning = False
        refresh_needed = True


def start_autotune():
    global abort_autotune
    if autotuning:
        return
    abort_autotune = False
    threading.Thread(target=autotune_worker, args=(target_var.get(),), daemon=True).start()



# ---------------- IMPORT / RESET / STEP TEST ----------------
def import_gains():
    """Pick a gains JSON file, load it and push it to all motors."""
    if autotuning:
        return
    path = filedialog.askopenfilename(
        title="Import gains (JSON)", filetypes=[("JSON", "*.json"), ("All files", "*.*")],
        initialdir=os.path.dirname(os.path.abspath(GAINS_FILE)))
    if not path:
        return
    try:
        n = load_gains(path)
        if n == 0:
            set_status(f"Import failed: no gains found in {os.path.basename(path)}")
            return
        for m in MOTOR_KEYS:
            push_gains(m)
        sync_sliders()
        set_status(f"Imported {os.path.basename(path)} ({n} values) and applied to the motors")
    except Exception as e:
        set_status(f"Import failed: {e}")


def reset_gains():
    """Back to DEFAULT_GAINS on every motor (sent to the ESP32, file on disk untouched)."""
    if autotuning:
        return
    for m in MOTOR_KEYS:
        motor_params[m].update(DEFAULT_GAINS)
        motor_params[m].update({k + "r": v for k, v in DEFAULT_GAINS.items()})
        push_gains(m)
    sync_sliders()
    set_status("Gains reset to defaults (tuned_gains.json not modified)")


def step_test_worker():
    """Closed-loop step 0 -> +VERIFY_RPM and 0 -> -VERIFY_RPM on all motors with the CURRENT gains;
    prints metrics per motor and direction. Run after RESET and after IMPORT for a before/after comparison."""
    global autotuning, capture, refresh_needed
    autotuning = True
    try:
        for sign, tag, suf in ((1, "fwd", ""), (-1, "rev", "r")):
            ser.write(b"TA0\n")
            time.sleep(SETTLE_STOP)
            set_status(f"Step test {tag}: all motors 0 -> {sign * VERIFY_RPM:.0f} rpm")
            capture = []
            ser.write(f"TA{sign * VERIFY_RPM:.1f}\n".encode('utf-8'))
            time.sleep(TEST_DURATION)
            cap, capture = capture, None
            ser.write(b"TA0\n")
            if len(cap) < 0.8 * TEST_DURATION / SAMPLE_DT:
                raise RuntimeError("serial data lost during the step test")
            for m, key in MOTOR_KEYS.items():
                met = step_metrics([sign * smp[key] for smp in cap], VERIFY_RPM, SAMPLE_DT)
                rise = f"{met['rise90_s']:.2f}s" if met["rise90_s"] is not None else "never"
                pm = motor_params[m]
                print(f"step test {key} {tag}: overshoot {met['overshoot_pct']:.1f}%, rise90 {rise}, settle {met['settle_s']:.2f}s, "
                      f"ripple {met['ripple_pp']:.1f} rpm pp, final {sign * met['final_rpm']:.1f} rpm "
                      f"(Kp={pm['P' + suf]} Ki={pm['I' + suf]} F={pm['F' + suf]} B={pm['B' + suf]})")
            time.sleep(SETTLE_STOP)
        set_status("Step test done (forward and reverse), results in the console")
    except Exception as e:
        capture = None
        ser.write(b"TA0\n")
        set_status(f"Step test failed: {e}")
    finally:
        autotuning = False
        refresh_needed = True


def speed_sweep():
    """Closed-loop steps 0 -> +-speed on all motors at once, for every speed in SWEEP_SPEEDS, forward and reverse,
    with the CURRENT gains. Prints final rpm per motor and flags speeds where the motors disagree or miss the target.
    Returns the number of flagged speeds. Call with the wheels lifted (reverse included)."""
    global capture
    flagged = 0
    for sign, tag in ((1, "fwd"), (-1, "rev")):
        for spd in SWEEP_SPEEDS:
            if abort_autotune:
                return flagged
            ser.write(b"TA0\n")
            time.sleep(SETTLE_STOP)
            set_status(f"Speed sweep {tag}: all motors 0 -> {sign * spd:.0f} rpm")
            capture = []
            ser.write(f"TA{sign * spd:.1f}\n".encode('utf-8'))
            time.sleep(TEST_DURATION)
            cap, capture = capture, None
            ser.write(b"TA0\n")
            if len(cap) < 0.8 * TEST_DURATION / SAMPLE_DT:
                raise RuntimeError("serial data lost during the speed sweep")
            finals, parts = {}, []
            for m, key in MOTOR_KEYS.items():
                met = step_metrics([sign * smp[key] for smp in cap], spd, SAMPLE_DT)
                finals[key] = met["final_rpm"]
                rise = f"{met['rise90_s']:.2f}s" if met["rise90_s"] is not None else "never"
                parts.append(f"{key} {met['final_rpm']:5.1f} (t90 {rise}, os {met['overshoot_pct']:.0f}%)")
            spread = max(finals.values()) - min(finals.values())
            miss = max(abs(v - spd) for v in finals.values())
            bad = spread > SWEEP_SPREAD_RPM or miss > SWEEP_SPREAD_RPM
            flagged += bad
            print(f"sweep {tag} {sign * spd:+.0f} rpm: spread {spread:.1f}, worst miss {miss:.1f} rpm | "
                  + " | ".join(parts) + ("   <-- CHECK" if bad else ""))
    ser.write(b"TA0\n")
    time.sleep(SETTLE_STOP)
    return flagged


def speed_sweep_worker():
    global autotuning, capture, refresh_needed
    autotuning = True
    try:
        n = speed_sweep()
        set_status("Speed sweep done: all speeds OK" if n == 0 else f"Speed sweep done: {n} speed(s) flagged, see the console")
    except Exception as e:
        capture = None
        ser.write(b"TA0\n")
        set_status(f"Speed sweep failed: {e}")
    finally:
        autotuning = False
        refresh_needed = True


flashing = False


def reconnect_serial():
    """(Re)open the dashboard's serial port to the PID test firmware and push the current gains."""
    global ser
    try:
        ser.close()
    except Exception:
        pass
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)                  # the board resets when the port opens
    for m in MOTOR_KEYS:
        push_gains(m)


def flash_worker(env, to_rig):
    """Close the dashboard's port, run 'pio run -e <env> -t upload' in ROBOT_PROJECT, then (to_rig) reconnect.
    env=PIO_ENV_ROBOT saves the gains first (JSON + gains.h) so the robot firmware is built with them."""
    global flashing, autotuning
    autotuning = True              # blocks slider/test commands while the port is closed
    try:
        if not ROBOT_PROJECT or not os.path.isdir(ROBOT_PROJECT):
            set_status("Set ROBOT_PROJECT (PlatformIO project folder) in the .env file")
            return
        if not to_rig:
            save_gains()
        try:
            ser.close()
        except Exception:
            pass
        cmd = [os.path.expanduser(PIO_CMD), "run", "-e", env, "-t", "upload", "-d", ROBOT_PROJECT,
               "--upload-port", UPLOAD_PORT or SERIAL_PORT]
        print("flash: " + " ".join(cmd))
        set_status(f"Building and uploading environment '{env}'...")
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace")
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                print("pio: " + line)
                set_status(f"Uploading '{env}': " + line[:70])
        rc = proc.wait()
        if rc != 0:
            set_status(f"Upload of '{env}' FAILED (exit code {rc}), see the console")
            if to_rig:
                try:
                    reconnect_serial()
                except Exception:
                    pass
            return
        if to_rig:
            time.sleep(2)          # board boots into the new firmware
            reconnect_serial()
            set_status("PID test uploaded and the dashboard is connected again")
        else:
            set_status("Robot firmware uploaded with the new gains. The dashboard is disconnected: "
                       "UPLOAD PID TEST to tune again")
    except FileNotFoundError:
        set_status(f"'{PIO_CMD}' not found: set PIO_CMD in .env (VS Code PlatformIO: ~/.platformio/penv/bin/pio)")
    except Exception as e:
        set_status(f"Upload failed: {e}")
    finally:
        flashing = False
        autotuning = False


def start_flash(env, to_rig):
    global flashing
    if flashing or autotuning:
        return
    flashing = True
    threading.Thread(target=flash_worker, args=(env, to_rig), daemon=True).start()


def start_flash_robot():
    start_flash(PIO_ENV_ROBOT, False)


def start_flash_test():
    start_flash(PIO_ENV_TEST, True)


def start_reconnect():
    """Reopen the port after the firmware was changed outside the dashboard (e.g. from VS Code)."""
    global flashing
    if flashing or autotuning:
        return

    def work():
        global flashing
        try:
            reconnect_serial()
            set_status("Reconnected, gains pushed")
        except Exception as e:
            set_status(f"Reconnect failed: {e}")
        finally:
            flashing = False
    flashing = True
    threading.Thread(target=work, daemon=True).start()


def start_speed_sweep():
    global abort_autotune
    if autotuning:
        return
    abort_autotune = False
    threading.Thread(target=speed_sweep_worker, daemon=True).start()


def start_step_test():
    if autotuning:
        return
    threading.Thread(target=step_test_worker, daemon=True).start()


# ---------------- PLOT / SERIAL ----------------
def update_plot():
    global refresh_needed
    status_var.set(status_text)
    if refresh_needed:
        refresh_needed = False
        sync_sliders()
    if not is_paused:
        ax.clear()
        ax.set_ylim(-150, 150)   
        ax.set_title("Live Motor RPM")
        ax.set_ylabel("RPM")
        if len(data['Target']) > 0:
            ax.plot(data['Target'], label='Target', color='black', linestyle='--')
            ax.plot(data['FL'], label='FL (Red)', color='red')
            ax.plot(data['FR'], label='FR (Green)', color='green')
            ax.plot(data['RL'], label='RL (Blue)', color='blue')
            ax.plot(data['RR'], label='RR (Orange)', color='orange')
            ax.legend(loc='upper left')
        canvas.draw()
    root.after(100, update_plot)


def read_serial():
    while True:
        try:
            if not ser.is_open or not ser.in_waiting:
                time.sleep(0.002)  # the original loop pegged a CPU core
                continue
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:          # port closed/replaced while uploading firmware: just wait
            time.sleep(0.05)
            continue
        try:
            parts = line.split()
            if len(parts) == 5 and "Target" in parts[0]:
                sample = {}
                for part in parts:
                    key, val = part.split(':')
                    sample[key] = float(val)
                for key, val in sample.items():
                    data[key].append(val)
                cap = capture
                if cap is not None:
                    cap.append(sample)
        except Exception:
            pass


# --- GUI SETUP ---
root = tk.Tk()
root.title("ESP32-S3 PID Tuning Dashboard")
root.geometry("1000x700")

ctrl_frame = tk.Frame(root, width=350)
ctrl_frame.pack(side=tk.LEFT, padx=20, fill=tk.Y)

tk.Label(ctrl_frame, text="Select Motor to Tune:", font=("Arial", 10, "bold")).pack(pady=(15, 5))
target_var = tk.StringVar(value="A")
tk.Radiobutton(ctrl_frame, text="ALL Motors (target RPM only)", variable=target_var, value="A", command=on_motor_select).pack(anchor="w")
tk.Radiobutton(ctrl_frame, text="FL (Front Left - Red)", variable=target_var, value="1", command=on_motor_select).pack(anchor="w")
tk.Radiobutton(ctrl_frame, text="FR (Front Right - Green)", variable=target_var, value="2", command=on_motor_select).pack(anchor="w")
tk.Radiobutton(ctrl_frame, text="RL (Rear Left - Blue)", variable=target_var, value="3", command=on_motor_select).pack(anchor="w")
tk.Radiobutton(ctrl_frame, text="RR (Rear Right - Orange)", variable=target_var, value="4", command=on_motor_select).pack(anchor="w")

tk.Label(ctrl_frame, text="Gain sliders edit:", font=("Arial", 10, "bold")).pack(pady=(10, 0))
dir_var = tk.StringVar(value="F")
dir_frame = tk.Frame(ctrl_frame)
dir_frame.pack()
tk.Radiobutton(dir_frame, text="Forward gains", variable=dir_var, value="F", command=on_motor_select).pack(side=tk.LEFT)
tk.Radiobutton(dir_frame, text="Reverse gains", variable=dir_var, value="R", command=on_motor_select).pack(side=tk.LEFT)

slider_len = 300

tk.Label(ctrl_frame, text="Target RPM").pack(pady=(15, 0))
rpm_slider = tk.Scale(ctrl_frame, from_=-120, to=120, length=slider_len, orient=tk.HORIZONTAL, command=lambda v: send_command('T', v))
rpm_slider.pack()

rpm_entry = tk.Entry(ctrl_frame, justify='center', width=20)
rpm_entry.pack(pady=5)
rpm_entry.bind('<Return>', lambda e: rpm_slider.set(float(rpm_entry.get())))

tk.Label(ctrl_frame, text="Proportional (Kp)").pack(pady=(10, 0))
p_slider = tk.Scale(ctrl_frame, from_=0.0, to=20.0, resolution=0.1, length=slider_len, orient=tk.HORIZONTAL, command=lambda v: send_command('P', v))
p_slider.pack()

tk.Label(ctrl_frame, text="Integral (Ki)").pack(pady=(10, 0))
i_slider = tk.Scale(ctrl_frame, from_=0.0, to=60.0, resolution=0.1, length=slider_len, orient=tk.HORIZONTAL, command=lambda v: send_command('I', v))
i_slider.pack()

tk.Label(ctrl_frame, text="Derivative (Kd)").pack(pady=(10, 0))
d_slider = tk.Scale(ctrl_frame, from_=0.0, to=2.0, resolution=0.05, length=slider_len, orient=tk.HORIZONTAL, command=lambda v: send_command('D', v))
d_slider.pack()

btn_frame = tk.Frame(ctrl_frame)
btn_frame.pack(pady=(20, 5))
pause_btn = tk.Button(btn_frame, text="⏸ PAUSE PLOT", font=("Arial", 10, "bold"), width=15, command=toggle_pause)
pause_btn.pack(side=tk.LEFT, padx=5)
stop_btn = tk.Button(btn_frame, text="STOP MOTORS", bg="red", fg="white", font=("Arial", 10, "bold"), width=15, command=stop_all_motors)
stop_btn.pack(side=tk.LEFT, padx=5)

btn_frame2 = tk.Frame(ctrl_frame)
btn_frame2.pack(pady=5)
tk.Button(btn_frame2, text="AUTOTUNE (selected / all)", font=("Arial", 10, "bold"), width=22, command=start_autotune).pack(side=tk.LEFT, padx=5)
tk.Button(btn_frame2, text="SAVE GAINS", font=("Arial", 10, "bold"), width=12, command=save_gains).pack(side=tk.LEFT, padx=5)

btn_frame3 = tk.Frame(ctrl_frame)
btn_frame3.pack(pady=5)
tk.Button(btn_frame3, text="IMPORT GAINS...", font=("Arial", 10, "bold"), width=15, command=import_gains).pack(side=tk.LEFT, padx=5)
tk.Button(btn_frame3, text="RESET TO DEFAULT", font=("Arial", 10, "bold"), width=15, command=reset_gains).pack(side=tk.LEFT, padx=5)

btn_frame4 = tk.Frame(ctrl_frame)
btn_frame4.pack(pady=5)
tk.Button(btn_frame4, text="STEP TEST (all motors)", font=("Arial", 10, "bold"), width=32, command=start_step_test).pack()
tk.Button(btn_frame4, text="SPEED SWEEP (10-120 rpm, fwd+rev)", font=("Arial", 10, "bold"), width=32, command=start_speed_sweep).pack(pady=(5, 0))
tk.Button(btn_frame4, text="UPLOAD PID TEST (then tune)", font=("Arial", 10, "bold"), width=32, bg="#ddddee", command=start_flash_test).pack(pady=(5, 0))
tk.Button(btn_frame4, text="SAVE + UPLOAD ROBOT FIRMWARE", font=("Arial", 10, "bold"), width=32, bg="#ddeedd", command=start_flash_robot).pack(pady=(5, 0))
tk.Button(btn_frame4, text="RECONNECT", font=("Arial", 10, "bold"), width=32, command=start_reconnect).pack(pady=(5, 0))

status_var = tk.StringVar(value=status_text)
tk.Label(ctrl_frame, textvariable=status_var, wraplength=330, justify="left", fg="#333").pack(pady=10)

fig, ax = plt.subplots(figsize=(7, 5))
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

sync_sliders()

threading.Thread(target=read_serial, daemon=True).start()
root.after(100, update_plot)
root.mainloop()