// ============================================================================
// CALIBRATE.cpp - interactive calibration over serial (115200 baud).
// Standalone test firmware, NOT part of the robot firmware. Needs pins.h and gains.h next to it.
// Motor gains are used as-is (same PI loop as main.cpp), never changed.
//
// platformio.ini:
//   [env:calibrate]
//   extends = env:<your normal env>
//   monitor_speed = 115200
//   build_src_filter = -<*> +<CALIBRATE.cpp>
//
// Apply the MPU6050_Custom::begin() clock fix first (PWR_MGMT_1 = 0x01), this file already uses it.
// Serial monitor must send a newline after each command. Any key sent while a test runs aborts it.
//
// Recommended order: b, d, g, k, t, w, s, r, m   (paste the printed constants into the firmware)
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Encoder.h>
#include "pins.h"
#include "gains.h"

// ---------------- must match main.cpp ----------------
static const float WHEEL_RADIUS   = 0.0381f;
static const float TICKS_PER_REV  = 748.0f;
static const float WHEEL_CIRC     = 2.0f * PI * WHEEL_RADIUS;
static const float M_PER_TICK     = WHEEL_CIRC / TICKS_PER_REV;
static const float MAX_WHEEL_RPM  = 120.0f;
static const float MAX_WHEEL_SPEED = MAX_WHEEL_RPM / 60.0f * WHEEL_CIRC;
#define SPEED_LOOP_MS   50
#define MOTOR_MAX_PWM   250
#ifndef I2C_CLOCK_HZ
#define I2C_CLOCK_HZ 100000
#endif
#define IMU_ADDR 0x68
static const float G0 = 9.80665f;

// ---------------- values under test (change at runtime: k / w / r) ----------------
static float gyro_scale    = 1.0f;      // GYRO_Z_SCALE in MPU6050_Custom.h
static float track_cmd     = 0.30f;     // TRACK_CMD in main.cpp
static float straight_trim = 0.0139f;   // STRAIGHT_TRIM in main.cpp

// ---------------- motors: same PI loop as main.cpp ----------------
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;

static void set_motor_pwm(int in1, int in2, int pwm_signed) {
  int pwm = constrain(abs(pwm_signed), 0, 255);
  if (pwm == 0) {
    digitalWrite(in2, LOW); analogWrite(in1, 0);
  } else if (pwm_signed > 0) {
    digitalWrite(in2, LOW); analogWrite(in1, pwm);
  } else {
    digitalWrite(in2, HIGH); analogWrite(in1, 255 - pwm);
  }
}

struct WheelCtrl {
  const MotorGains* g;
  ESP32Encoder* enc;
  float dir;
  int in1, in2;
  int32_t last_count;
  float target_rpm, integ, meas_rpm;
  int8_t last_sign;
};

WheelCtrl wheels[4] = {   // FL, RL, FR, RR
  {&GAINS_FL, &enc_fl, (float)MOTOR_DIR_FL, FL_IN1, FL_IN2, 0, 0, 0, 0, 0},
  {&GAINS_RL, &enc_rl, (float)MOTOR_DIR_RL, RL_IN1, RL_IN2, 0, 0, 0, 0, 0},
  {&GAINS_FR, &enc_fr, (float)MOTOR_DIR_FR, FR_IN1, FR_IN2, 0, 0, 0, 0, 0},
  {&GAINS_RR, &enc_rr, (float)MOTOR_DIR_RR, RR_IN1, RR_IN2, 0, 0, 0, 0, 0},
};

static void speed_ctrl_reset() {
  for (auto &w : wheels) {
    w.integ = 0.0f; w.meas_rpm = 0.0f; w.last_sign = 0;
    w.last_count = (int32_t)w.enc->getCount();
  }
}

static void speed_ctrl_stop() {
  for (auto &w : wheels) {
    w.target_rpm = 0.0f; w.integ = 0.0f;
    set_motor_pwm(w.in1, w.in2, 0);
  }
}

static void speed_ctrl_step(float dt) {
  for (auto &w : wheels) {
    int32_t c = (int32_t)w.enc->getCount();
    float rpm = (float)(w.dir * (c - w.last_count)) / TICKS_PER_REV * 60.0f / dt;
    w.last_count = c;
    w.meas_rpm = rpm;

    float r = w.target_rpm;
    if (fabsf(r) < 0.5f) { w.integ = 0.0f; set_motor_pwm(w.in1, w.in2, 0); continue; }

    const Gain& g = (r > 0.0f) ? w.g->fwd : w.g->rev;
    int8_t sgn = (r > 0.0f) ? 1 : -1;
    if (sgn != w.last_sign) { w.integ = 0.0f; w.last_sign = sgn; }

    float e  = r - rpm;
    float ff = g.ff * r + sgn * g.off;
    float u_pre = ff + g.kp * e + w.integ;
    bool saturated = fabsf(u_pre) >= MOTOR_MAX_PWM;
    if (!(saturated && e * u_pre > 0.0f)) w.integ += g.ki * dt * e;
    w.integ = constrain(w.integ, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);

    float u = constrain(ff + g.kp * e + w.integ, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
    if (u * r < 0.0f) u = 0.0f;
    set_motor_pwm(w.in1, w.in2, (int)lroundf(u * w.dir));
  }
}

static void drive_motors(float left_speed, float right_speed) {
  float l = constrain(left_speed,  -1.0f, 1.0f);
  float r = constrain(right_speed, -1.0f, 1.0f);
  wheels[0].target_rpm = l * MAX_WHEEL_RPM;
  wheels[1].target_rpm = l * MAX_WHEEL_RPM;
  wheels[2].target_rpm = r * MAX_WHEEL_RPM;
  wheels[3].target_rpm = r * MAX_WHEEL_RPM;
  if (fabsf(l) < 0.01f && fabsf(r) < 0.01f) speed_ctrl_stop();
}

// Same mixing as main.cpp apply_drive() (trim passed in so it can be tested)
static void apply_drive(float lin, float ang, float trim) {
  float w  = ang - trim * lin;
  float vl = lin - w * track_cmd * 0.5f;
  float vr = lin + w * track_cmd * 0.5f;
  float m  = fmaxf(fabsf(vl), fabsf(vr));
  if (m > MAX_WHEEL_SPEED) { float k = MAX_WHEEL_SPEED / m; vl *= k; vr *= k; }
  drive_motors(vl / MAX_WHEEL_SPEED, vr / MAX_WHEEL_SPEED);
}

// ---------------- IMU (raw, no offsets: this measures the offsets) ----------------
static bool imu_write(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}

static bool imu_init() {
  if (!imu_write(0x6B, 0x01)) return false;   // wake, gyro-X PLL clock
  delay(100);
  return imu_write(0x1A, 0x03)                // DLPF ~44 Hz
      && imu_write(0x1B, 0x00)                // gyro +-250 dps  (131 LSB per dps)
      && imu_write(0x1C, 0x00);               // accel +-2 g     (16384 LSB per g)
}

// a in m/s^2, g in rad/s, raw (no offsets, no scale factor)
static bool imu_read(float a[3], float g[3]) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)IMU_ADDR, 14, true) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  if ((float)v[0] * v[0] + (float)v[1] * v[1] + (float)v[2] * v[2] < 6.7e7f) return false;   // asleep / reset
  for (int i = 0; i < 3; i++) {
    a[i] = v[i] / 16384.0f * G0;
    g[i] = v[4 + i] / 131.0f * (PI / 180.0f);
  }
  return true;
}

// ---------------- service loop: 20 Hz speed PI + 200 Hz IMU ----------------
static uint32_t last_speed_ms = 0, last_imu_us = 0;
static void service_reset() { last_speed_ms = millis(); last_imu_us = micros(); }

struct Sample { bool ok = false; bool pi = false; float dt = 0; float a[3] = {0, 0, 0}; float g[3] = {0, 0, 0}; };

static Sample service() {
  Sample s;
  uint32_t ms = millis();
  if (ms - last_speed_ms >= SPEED_LOOP_MS) {
    float dt = (ms - last_speed_ms) * 1e-3f;
    last_speed_ms = ms;
    if (dt > 0.25f) speed_ctrl_reset(); else { speed_ctrl_step(dt); s.pi = true; }
  }
  uint32_t us = micros();
  if (us - last_imu_us >= 5000) {
    float dt = (us - last_imu_us) * 1e-6f;
    last_imu_us = us;
    if (dt < 0.05f && imu_read(s.a, s.g)) { s.ok = true; s.dt = dt; }
  }
  return s;
}

static bool key_pressed() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

static bool hold(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (key_pressed()) { drive_motors(0, 0); return false; }
    service();
  }
  return true;
}

struct Stats { double ma[3], mg[3], sg[3]; int n; };

// Robot must be still. Averages accel and gyro for ms milliseconds.
static bool collect_still(uint32_t ms, Stats &st) {
  double sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0}, sg2[3] = {0, 0, 0};
  int n = 0;
  drive_motors(0, 0);
  service_reset();
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (key_pressed()) return false;
    Sample s = service();
    if (!s.ok) continue;
    for (int i = 0; i < 3; i++) { sa[i] += s.a[i]; sg[i] += s.g[i]; sg2[i] += (double)s.g[i] * s.g[i]; }
    n++;
  }
  if (n < 50) { Serial.println("IMU not delivering samples, check wiring."); return false; }
  for (int i = 0; i < 3; i++) {
    st.ma[i] = sa[i] / n;
    st.mg[i] = sg[i] / n;
    double var = sg2[i] / n - st.mg[i] * st.mg[i];
    st.sg[i] = sqrt(var > 0 ? var : 0);
  }
  st.n = n;
  return true;
}

// ---------------- motion run: mean yaw rate and mean forward speed after a settle time ----------------
struct Run { double gz = 0, v = 0; bool ok = false; };

static Run run_motion(float lin, float ang, float trim, float total_s, float settle_s, float bias) {
  Run r;
  double sgz = 0, sv = 0;
  int ng = 0, nv = 0;
  speed_ctrl_reset();
  service_reset();
  apply_drive(lin, ang, trim);
  uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)(total_s * 1000.0f)) {
    if (key_pressed()) { drive_motors(0, 0); return r; }
    Sample s = service();
    if ((millis() - t0) * 1e-3f < settle_s) continue;
    if (s.ok) { sgz += (s.g[2] - bias) * gyro_scale; ng++; }
    if (s.pi) {
      float rpm = 0;
      for (auto &w : wheels) rpm += w.meas_rpm;
      sv += (rpm / 4.0f) / 60.0f * WHEEL_CIRC;
      nv++;
    }
  }
  drive_motors(0, 0);
  if (ng < 20 || nv < 5) return r;
  r.gz = sgz / ng;
  r.v  = sv / nv;
  r.ok = true;
  return r;
}

static bool get_bias(float &bias) {
  Serial.println("Keep the robot still (3 s)...");
  Stats st;
  if (!collect_still(3000, st)) return false;
  bias = (float)st.mg[2];
  return true;
}

// ---------------- commands ----------------
static void cmd_bias() {
  Serial.println("Robot still and level, 5 s...");
  Stats st;
  if (!collect_still(5000, st)) { Serial.println("aborted"); return; }
  float amag = sqrtf((float)(st.ma[0] * st.ma[0] + st.ma[1] * st.ma[1] + st.ma[2] * st.ma[2]));
  Serial.printf("samples %d\n", st.n);
  Serial.printf("gyro mean  x %.5f  y %.5f  z %.5f rad/s   (sigma z %.5f)\n", st.mg[0], st.mg[1], st.mg[2], st.sg[2]);
  Serial.printf("accel mean x %.4f  y %.4f  z %.4f m/s^2  |a| %.4f (1 g = %.4f)\n", st.ma[0], st.ma[1], st.ma[2], amag, G0);
  if (fabsf((float)st.ma[0]) > 1.0f || fabsf((float)st.ma[1]) > 1.0f)
    Serial.println("WARNING: robot not level or IMU tilted > ~6 deg. Accel offsets below would hide the tilt, skip them.");
  Serial.println("Paste into MPU6050_Custom.cpp readData() (accel offsets are optional, attitude.h already handles tilt and gain):");
  Serial.printf("  ax = (v[0] / 16384.0f * G) - (%.4ff);\n", st.ma[0]);
  Serial.printf("  ay = (v[1] / 16384.0f * G) - (%.4ff);\n", st.ma[1]);
  Serial.printf("  az = (v[2] / 16384.0f * G) - (%.4ff);\n", st.ma[2] - G0);
  Serial.printf("  gx = ((v[4] / 131.0f) * (PI / 180.0f)) - (%.5ff);\n", st.mg[0]);
  Serial.printf("  gy = ((v[5] / 131.0f) * (PI / 180.0f)) - (%.5ff);\n", st.mg[1]);
}

static void cmd_drift(float seconds) {
  if (seconds < 10) seconds = 60;
  float bias;
  if (!get_bias(bias)) { Serial.println("aborted"); return; }
  Serial.printf("Integrating gyro z for %.0f s, keep still...\n", seconds);
  service_reset();
  double ang = 0;
  uint32_t t0 = millis(), last_print = 0;
  while ((millis() - t0) * 1e-3f < seconds) {
    if (key_pressed()) { Serial.println("aborted"); return; }
    Sample s = service();
    if (s.ok) ang += (s.g[2] - bias) * s.dt;
    if (millis() - last_print >= 10000) { last_print = millis(); Serial.printf("  %.0f s: %.3f deg\n", (millis() - t0) * 1e-3f, ang * 180.0 / PI); }
  }
  Serial.printf("drift %.3f deg total = %.3f deg/min = %.5f rad/s mean residual bias\n",
                ang * 180.0 / PI, ang * 180.0 / PI * 60.0 / seconds, ang / seconds);
  Serial.println("main.cpp clamps the learned bias to +-0.01 rad/s around the boot value. If the residual here is far smaller, tighten the clamp.");
}

static void cmd_gyro_scale(float target_deg) {
  if (target_deg < 1) target_deg = 360;
  float bias;
  if (!get_bias(bias)) { Serial.println("aborted"); return; }
  Serial.printf("ROTATE the robot by hand, flat on the floor, through exactly %.0f deg (use tape marks), smoothly.\n", target_deg);
  Serial.println("Send any key when finished.");
  service_reset();
  double ang = 0;
  uint32_t last_print = millis();
  for (;;) {
    if (key_pressed()) break;
    Sample s = service();
    if (s.ok) ang += (s.g[2] - bias) * s.dt;
    if (millis() - last_print >= 1000) { last_print = millis(); Serial.printf("  %.1f deg\n", ang * 180.0 / PI); }
  }
  double meas = fabs(ang * 180.0 / PI);
  if (meas < 10) { Serial.println("Measured < 10 deg, no valid rotation."); return; }
  Serial.printf("measured %.2f deg for %.0f deg true\n", meas, target_deg);
  Serial.printf("GYRO_Z_SCALE = %.5f   (MPU6050_Custom.h; set it here with: k %.5f)\n", target_deg / meas, target_deg / meas);
  Serial.println("Repeat once each direction (CW and CCW) and average the two scales.");
}

static void cmd_track(float wcmd) {
  if (wcmd <= 0) wcmd = 1.0f;
  float bias;
  if (!get_bias(bias)) { Serial.println("aborted"); return; }
  Serial.printf("Spinning in place at +-%.2f rad/s, clear 1 m around. Gyro scale %.5f, TRACK_CMD %.4f\n", wcmd, gyro_scale, track_cmd);
  Run a = run_motion(0, wcmd, 0, 6.0f, 2.5f, bias);
  if (!a.ok) { Serial.println("aborted"); return; }
  delay(1500);
  Run b = run_motion(0, -wcmd, 0, 6.0f, 2.5f, bias);
  if (!b.ok) { Serial.println("aborted"); return; }
  double mp = a.gz, mn = -b.gz;
  double mean = (mp + mn) / 2.0;
  Serial.printf("measured CCW %.4f rad/s, CW %.4f rad/s (commanded %.2f)\n", mp, mn, wcmd);
  if (mean < 0.05) { Serial.println("Measured rate too small, check gyro scale and motors."); return; }
  double nt = track_cmd * wcmd / mean;
  Serial.printf("TRACK_CMD = %.4f   (main.cpp; set it here with: w %.4f)\n", nt, nt);
  if (fabs(mp - mn) > 0.1 * mean) Serial.println("CCW and CW differ by > 10%: surface, battery or wheel asymmetry.");
  Serial.println("Run 'g' and 'k' first if the gyro scale is not calibrated yet.");
}

static void cmd_trim(float v) {
  if (v <= 0) v = 0.23f;
  float bias;
  if (!get_bias(bias)) { Serial.println("aborted"); return; }
  Serial.printf("Driving straight at %.2f m/s forward then reverse (about %.1f m each, clear the path). Trim in use %.5f\n", v, v * 8.0f, straight_trim);
  Run f = run_motion(v, 0, straight_trim, 8.0f, 2.5f, bias);
  if (!f.ok) { Serial.println("aborted"); return; }
  delay(1500);
  Run r = run_motion(-v, 0, straight_trim, 8.0f, 2.5f, bias);
  if (!r.ok) { Serial.println("aborted"); return; }
  double vf = fabs(f.v), vr = fabs(r.v);
  if (vf < 0.02 || vr < 0.02) { Serial.println("Robot barely moved, aborting calculation."); return; }
  Serial.printf("residual yaw rate: forward %.5f rad/s, reverse %.5f rad/s  (+ = left)\n", f.gz, r.gz);
  double tf = straight_trim + f.gz / vf;
  double tr = straight_trim - r.gz / vr;
  double t  = (tf + tr) / 2.0;
  Serial.printf("trim from forward %.5f, from reverse %.5f\n", tf, tr);
  Serial.printf("STRAIGHT_TRIM = %.5f   (main.cpp; set it here with: r %.5f, then run s again to verify, residual should be ~0)\n", t, t);
  if (fabs(tf - tr) > 0.01) Serial.println("Forward and reverse disagree by > 0.01: not a pure bias, run again on a clean floor.");
}

static void cmd_minspeed(float vmax) {
  if (vmax <= 0) vmax = 0.08f;
  Serial.println("Linear sweep, each step starts from rest (1 s settle + 1 s measure). Clear ~1 m ahead.");
  Serial.println("target m/s   measured m/s   ratio");
  float first_lin = -1;
  for (float v = 0.01f; v <= vmax + 1e-4f; v += 0.01f) {
    Run r = run_motion(v, 0, 0, 2.0f, 1.0f, 0);
    if (!r.ok) { Serial.println("aborted"); return; }
    float ratio = (float)r.v / v;
    Serial.printf("%.3f        %.4f         %.2f\n", v, r.v, ratio);
    if (first_lin < 0 && ratio >= 0.7f) first_lin = v;
    delay(300);
  }
  if (first_lin > 0) Serial.printf("lowest tracked linear speed (>= 70%% of target): %.3f m/s\n", first_lin);
  else Serial.println("no step tracked within 70% of target");

  float bias;
  if (!get_bias(bias)) { Serial.println("aborted"); return; }
  Serial.println("In-place rotation sweep.");
  Serial.println("target rad/s   measured rad/s   ratio");
  float first_rot = -1;
  for (float w = 0.05f; w <= 0.501f; w += 0.05f) {
    Run r = run_motion(0, w, 0, 2.0f, 1.0f, bias);
    if (!r.ok) { Serial.println("aborted"); return; }
    float ratio = (float)r.gz / w;
    Serial.printf("%.2f         %.4f          %.2f\n", w, r.gz, ratio);
    if (first_rot < 0 && ratio >= 0.7f) first_rot = w;
    delay(300);
  }
  if (first_rot > 0) Serial.printf("lowest tracked rotation (>= 70%% of target): %.2f rad/s\n", first_rot);
  else Serial.println("no step tracked within 70% of target");
  Serial.println("Below these values the robot stalls or under-turns. Nav2 commands under them will not be executed faithfully.");
}

static void print_settings() {
  Serial.printf("gyro_scale (k) %.5f | track_cmd (w) %.4f | straight_trim (r) %.5f\n", gyro_scale, track_cmd, straight_trim);
}

static void print_help() {
  Serial.println("\n=== CALIBRATE ===");
  Serial.println("b            still+level: gyro offsets, accel offsets, noise");
  Serial.println("d [sec]      still: gyro drift (default 60)");
  Serial.println("g [deg]      gyro z scale, rotate by hand (default 360)");
  Serial.println("t [rad/s]    TRACK_CMD, spins in place both ways (default 1.0)");
  Serial.println("s [m/s]      STRAIGHT_TRIM, drives straight fwd+rev (default 0.23)");
  Serial.println("m [max m/s]  minimum tracked speed, linear + rotation (default 0.08)");
  Serial.println("k/w/r <val>  set gyro scale / TRACK_CMD / STRAIGHT_TRIM for the tests");
  Serial.println("p            print current values");
  Serial.println("x            stop motors");
  print_settings();
}

static void handle(String line) {
  line.trim();
  if (!line.length()) return;
  char c = line[0];
  float arg = line.substring(1).toFloat();
  switch (c) {
    case 'b': cmd_bias(); break;
    case 'd': cmd_drift(arg); break;
    case 'g': cmd_gyro_scale(arg); break;
    case 't': cmd_track(arg); break;
    case 's': cmd_trim(arg); break;
    case 'm': cmd_minspeed(arg); break;
    case 'k': if (arg > 0.5f && arg < 1.5f) gyro_scale = arg; print_settings(); break;
    case 'w': if (arg > 0.1f && arg < 1.0f) track_cmd = arg; print_settings(); break;
    case 'r': straight_trim = arg; print_settings(); break;
    case 'p': print_settings(); break;
    case 'x': drive_motors(0, 0); Serial.println("stopped"); break;
    default:  print_help(); break;
  }
  Serial.println("done");
}

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(20000);

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  drive_motors(0, 0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B); enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B); enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(5);
  delay(1500);   // let the serial monitor attach
  if (!imu_init()) Serial.println("IMU init FAILED (check wiring / address)");
  speed_ctrl_reset();
  print_help();
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) { String l = line; line = ""; handle(l); }
    } else {
      line += c;
    }
  }
  drive_motors(0, 0);   // idle: motors off
}
