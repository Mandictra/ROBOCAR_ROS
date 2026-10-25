#include <Arduino.h>
#include <SPI.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>

#include <ESP32Encoder.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <BasicLinearAlgebra.h>

#include "pins.h"
#include "MPU6050_Custom.h"
#include "attitude.h"
#include "gains.h"   // per-wheel PI gains generated from tuned_gains.json (json_to_gains_h.py)

using namespace BLA;

// ================= ROS 2 topics (node "base_controller", no namespace) =================
// Published by the ESP32 (all best effort QoS, so subscribers must also use best effort or sensor-data QoS):
//   sensors/wheel_ticks  std_msgs/Int32MultiArray  20 Hz  [FL, RL, FR, RR ticks, stamp_sec, stamp_nanosec]
//   imu_data             sensor_msgs/Imu           20 Hz  frame imu_link; roll/pitch from attitude filter, yaw from gyro integration
//                                                         (orientation yaw covariance is 1e3: do not fuse it as an absolute heading)
//   odometry/filtered    nav_msgs/Odometry         20 Hz  frame odom -> child base_link; EKF pose + velocity (this is Nav2's odom)
//                                                         NO TF is broadcast by the ESP32: the RPi5 must publish odom -> base_link from this topic
//   sensors/color/rgb    geometry_msgs/Vector3     10 Hz  TCS34725 raw R, G, B
//   self_check           std_msgs/Int32             1 Hz  2 bits per item (0 not run, 1 pass, 2 fail, 3 check by eye), bit 30 = run toggle
// Subscribed by the ESP32 (default reliable QoS):
//   cmd_vel_raw          geometry_msgs/Twist       drive command in m/s and rad/s (from twist_mux); angular.x > 0.5 asks for heading hold
//   actuators/pan_cmd    std_msgs/Float32          pan SPEED -1..1 (continuous-rotation servo, PCA9685 channel 0); keep publishing,
//                                                  it stops CMD_TIMEOUT_MS after the last non-zero command
//   actuators/tilt_cmd   geometry_msgs/Point       y = tilt servo angle in degrees (x is ignored; PCA9685 channel 1)
//   actuators/dump_cmd   std_msgs/Float32          dump servo angle, 15..165 degrees (PCA9685 channel 2)
//   actuators/conveyor_cmd std_msgs/Float32        conveyor stepper speed in steps/s (+-700)
//   self_check_cmd       std_msgs/Int32            any message starts the self-check (ignored while driving)
//   heading_hold_cmd     std_msgs/Bool             master switch for heading hold (true again after every agent reconnect)
// Timestamps (wheel_ticks, imu_data, odometry/filtered) are agent-synchronised epoch time. Nothing stamped is published
// until the time sync with the agent has succeeded, so the agent must run on the same machine/clock as the lidar driver.
// =========================================================================================

// ---------- Robot Kinematics (Tune these in meters) ----------
const float WHEEL_RADIUS = 0.0381f; // 76.2 mm wheel diameter (verified by distance test)
const float TRACK_WIDTH = 0.215f;   // geometric track, informational only (turn commands and encoder yaw use TRACK_CMD)
const float TICKS_PER_REV = 748.0f; // Half-quad counts per wheel rev
const float M_PER_TICK = (2.0f * PI * WHEEL_RADIUS) / TICKS_PER_REV;

// ---------- Wheel speed PI ----------
#define SPEED_LOOP_MS 50            // must equal the sample time the gains were tuned at (SAMPLE_DT = 0.05 s)
const float MAX_WHEEL_RPM = 120.0f;  // wheel rpm at normalised speed 1.0 (tuning dashboard range is 0..120 rpm)

// ---------- cmd_vel is in real units (m/s, rad/s) so Nav2 can drive it directly ----------
const float WHEEL_CIRC      = 2.0f * PI * WHEEL_RADIUS;            // 0.2394 m
const float MAX_WHEEL_SPEED = MAX_WHEEL_RPM / 60.0f * WHEEL_CIRC;  // 0.479 m/s at 120 rpm
// Effective track for turn commands. Skid steer turns as if the track were wider than the geometric 0.215 m (TRACK_WIDTH).
// Tune: command 1.0 rad/s, read /imu_data angular_velocity.z, TRACK_CMD_new = TRACK_CMD * 1.0 / measured.
const float TRACK_CMD = 0.30f;
// Straight-line trim (1/m). Measured veer to the LEFT of 0.0032 rad/s at 0.23 m/s -> 0.0032/0.23 = 0.0139 1/m.
// Added as a yaw-rate correction proportional to forward speed: w -= STRAIGHT_TRIM * cmd_lin (works forward and reverse).
// Tune: drive 3 m straight at normal speed; still veers LEFT -> increase it, veers RIGHT -> decrease it, straight -> done. 0 = off.
// (Only the no-IMU fallback yaw is slightly affected: its encoder yaw then reads this deliberate wheel difference.)
const float STRAIGHT_TRIM = 0.0139f;
// Cap on the commanded turn rate. Keeps lidar scans clean for SLAM; keep dashboard WMAX and Nav2 max angular <= this.
const float MAX_YAW_RATE = 2.0f;   // rad/s
// Fraction of linear speed removed at full turn rate. 0 = pass cmd_vel through unchanged (right for Nav2, so its
// predicted trajectories match what the robot does). The old behaviour was 0.5 (set it back if the dashboard needs it).
#define TURN_SLOWDOWN 0.0f

// Wheel labels are from the robot's own perspective (standing behind it, looking the way it drives).
// Sign that makes each wheel's encoder count positive when the robot rolls forward.
// Encoder sign for odometry (counts positive when the wheel rolls forward). It must equal MOTOR_DIR_*: the encoders count
// up when a motor is driven with positive raw PWM (that is what PID_TEST.cpp relies on), and forward is raw sign MOTOR_DIR_*.
// Hard-coding it separately (it was -1,-1,+1,+1) disagreed with pins.h on the FRONT wheels.
const int S_FL = MOTOR_DIR_FL, S_RL = MOTOR_DIR_RL, S_FR = MOTOR_DIR_FR, S_RR = MOTOR_DIR_RR;

// ---------- EKF Matrices (4-State) ----------
// State X = [x, y, theta, v]. Prediction: gyro for theta, gravity-compensated accelerometer for v.
// Measurement: encoder forward velocity only (H = [0 0 0 1]). x, y and theta are dead-reckoned, so their covariance
// grows with distance/time instead of being reset by fake measurements.
BLA::Matrix<4> X = {0.0f, 0.0f, 0.0f, 0.0f}; // State vector [x, y, theta, v]
BLA::Matrix<4, 4> P = {1.0f, 0.0f, 0.0f, 0.0f,
                       0.0f, 1.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 1.0f, 0.0f,
                       0.0f, 0.0f, 0.0f, 1.0f}; // Covariance Matrix

// Process noise per 50 ms step. Q(0,0)/Q(1,1) = 1e-6 (~1 mm/step; position is integrated from the fused velocity). Q(2,2) = 1e-5 is my estimate for gyro-integrated yaw (the old 0.005 was far too pessimistic):
// tune it against measured yaw drift, e.g. a 10 minute still test and a few full-lap runs.
BLA::Matrix<4, 4> Q = {1e-6f,  0.0f,   0.0f,   0.0f,
                       0.0f,   1e-6f,  0.0f,   0.0f,
                       0.0f,   0.0f,   1e-5f,  0.0f,
                       0.0f,   0.0f,   0.0f,   0.05f}; // Process Noise (Gyro & Accel trust)

// Measurement noise: only R(3,3) (encoder velocity) is used. The other entries are kept for reference.
BLA::Matrix<4, 4> R = {0.05f, 0.0f,  0.0f,  0.0f,
                       0.0f,  0.05f, 0.0f,  0.0f,
                       0.0f,  0.0f,  0.1f,  0.0f,
                       0.0f,  0.0f,  0.0f,  0.02f}; // Measurement Noise (Encoder trust)

int32_t last_enc_l = 0;
int32_t last_enc_r = 0;

// ---------- Hardware Instances ----------
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;
AccelStepper conveyorStepper(AccelStepper::FULL4WIRE, CONV_IN1, CONV_IN3, CONV_IN2, CONV_IN4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
MPU6050_Custom imuSensor;
Attitude att;   // roll/pitch complementary filter + ramp flag (ROS FLU frame)

// ---------- IMU oversampling (200 Hz read, averaged down to the 50 ms estimator tick) ----------
static float    imu_acc[6] = {0, 0, 0, 0, 0, 0};   // ax ay az gx gy gz sums
static uint16_t imu_acc_n  = 0;
float gz_bias0 = 0.0f;                              // boot-calibrated gyro z bias, centre of the clamp used while driving

const int TCS_ADDR = 0x29;

// ---------- Servos on a PCA9685 (I2C 0x40, shares the bus with the IMU and colour sensor) ----------
// All PCA9685 access happens on core 1 (process_commands / selfcheck / setup). The micro-ROS core never touches Wire.
#define PCA_FREQ_HZ   50
// Absolute safety clamp for any pulse. The real per-servo limits are the SERVO_CAL table below (found with `find` in
// pca_serial_test.cpp); this only stops a typo in that table from sending a wild pulse.
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2650   // must be >= the largest us in SERVO_CAL (dump uses 2600), or that end gets clipped
#define PCA_REG_MODE1      0x00
#define PCA_REG_LED0_ON_L  0x06
#define PCA_REG_PRESCALE   0xFE

static bool pca_ok = false;

static bool pca_write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool pca_read8(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)PCA_ADDR, 1) != 1) return false;
  val = Wire.read();
  return true;
}

// Sets 50 Hz and auto-increment. Takes about 1 ms. The internal oscillator is only about +-5 % accurate.
static bool pca_init() {
  uint8_t prescale = (uint8_t)(25000000.0f / (4096.0f * PCA_FREQ_HZ) + 0.5f) - 1;   // 121
  if (!pca_write8(PCA_REG_MODE1, 0x10)) return false;       // sleep (prescale is writable only while asleep)
  if (!pca_write8(PCA_REG_PRESCALE, prescale)) return false;
  if (!pca_write8(PCA_REG_MODE1, 0x00)) return false;       // wake
  delayMicroseconds(500);                                   // oscillator settle time
  if (!pca_write8(PCA_REG_MODE1, 0xA0)) return false;       // restart + auto-increment
  uint8_t m = 0;
  return pca_read8(PCA_REG_MODE1, m) && (m & 0x20);
}

static bool pca_set_us(uint8_t ch, float us) {
  us = constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  uint16_t off = (uint16_t)(us * 4096.0f * PCA_FREQ_HZ / 1e6f + 0.5f);
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(PCA_REG_LED0_ON_L + 4 * ch);
  Wire.write(0); Wire.write(0);
  Wire.write(off & 0xFF); Wire.write(off >> 8);
  return Wire.endTransmission() == 0;
}

// Per-channel calibration: two reference points, command angle -> measured pulse. Angles in between are linearly
// interpolated; angles outside the two points are clamped to them, so a servo is never driven past its calibrated ends.
// The values below are PLACEHOLDERS: fill them from `find <ch>` in pca_serial_test.cpp, one servo at a time.
// If a servo turns the wrong way, swap its two us values. The command ranges (tilt 30..150, dump 15..165) stay as they were.
struct ServoCal { float deg0, us0, deg1, us1; };
static const ServoCal SERVO_CAL[3] = {
  {   0, 1000, 180, 2000 },   // CH_PAN unused: continuous-rotation servo, see pan_set_speed()
  {   0, 1602, 180, 2602 },   // CH_TILT  FINAL (measured). Commands are still limited to TILT_MIN..TILT_MAX (30..150 deg)
  {   0,  700, 180, 2600 },   // CH_DUMP  FINAL (measured), do not change. Commands are still limited to 15..165 deg
};

// Angle in degrees on a PCA9685 channel, interpolated through SERVO_CAL. Re-initialises the chip if it dropped off the bus.
static void servo_write(uint8_t ch, float deg) {
  if (ch == CH_PAN || ch >= 3) return;   // pan is continuous rotation: use pan_set_speed()
  if (!pca_ok && !(pca_ok = pca_init())) return;
  const ServoCal &c = SERVO_CAL[ch];
  float t = constrain((deg - c.deg0) / (c.deg1 - c.deg0), 0.0f, 1.0f);
  if (!pca_set_us(ch, c.us0 + t * (c.us1 - c.us0))) pca_ok = false;
}

// ---------- Pan: continuous-rotation servo (SG90 360), PCA9685 channel 0 ----------
// A continuous-rotation servo has no angle: the pulse sets speed and direction around a stop pulse, and nothing reports
// position. So pan is a speed command (-1..1), not part of SERVO_CAL. The sender must keep publishing while it wants
// motion: pan stops by itself CMD_TIMEOUT_MS after the last non-zero command (checked in loop()).
#define PAN_STOP_US   1500   // pulse where it should stand still
#define PAN_RANGE_US  400    // offset from PAN_STOP_US at |speed| = 1. Tune: CR servos saturate somewhere around +-300..500 us
#define PAN_DIR       1      // -1 reverses the direction
#define PAN_DEADBAND  0.05f  // |speed| below this = stop

static uint32_t pan_last_ms = 0;
static bool pan_active = false;

// Full-off bit: no pulses at all on the channel. A CR servo gets no signal, so it cannot creep from a badly trimmed stop pulse.
static bool pca_channel_off(uint8_t ch) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(PCA_REG_LED0_ON_L + 4 * ch);
  Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
  return Wire.endTransmission() == 0;
}

static void pan_set_speed(float s) {
  s = constrain(s, -1.0f, 1.0f) * PAN_DIR;
  bool moving = fabsf(s) >= PAN_DEADBAND;
  pan_active = moving;                         // set before any I2C so a dead bus cannot make loop() retry every pass
  if (moving) pan_last_ms = millis();
  if (!pca_ok && !(pca_ok = pca_init())) return;
  bool ok = moving ? pca_set_us(CH_PAN, PAN_STOP_US + s * PAN_RANGE_US) : pca_channel_off(CH_PAN);
  if (!ok) pca_ok = false;
}

// Average ~2 s of still samples and calibrate the attitude filter.
// Call after imuSensor.calibrateGyro() with the robot stationary on level ground.
static void attitudeCalibrate() {
  double a[3] = {0, 0, 0}, g[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < 100; i++) {
    float v[6];
    if (imuSensor.readData(v[0], v[1], v[2], v[3], v[4], v[5])) {
      for (int j = 0; j < 3; j++) { a[j] += v[j]; g[j] += v[3 + j]; }
      n++;
    }
    delay(20);
  }
  if (n < 50) return;
  float af[3], gf[3];
  for (int j = 0; j < 3; j++) { af[j] = a[j] / n; gf[j] = g[j] / n; }
  att.calibrate(af, gf);
}

// ---------- micro-ROS Entities ----------
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t twist_sub;
geometry_msgs__msg__Twist twist_msg;
rcl_subscription_t pan_sub;
std_msgs__msg__Float32 pan_msg;
rcl_subscription_t tilt_sub;
geometry_msgs__msg__Point tilt_msg;
rcl_subscription_t dump_sub;
std_msgs__msg__Float32 dump_msg;
rcl_subscription_t conv_sub;
std_msgs__msg__Float32 conv_msg;



rcl_subscription_t hold_sub;
std_msgs__msg__Bool hold_msg;

rcl_publisher_t enc_pub;
std_msgs__msg__Int32MultiArray enc_msg;
int32_t enc_data[6];

rcl_publisher_t color_pub;
geometry_msgs__msg__Vector3 color_msg;

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;
static char odom_frame[] = "odom";
static char base_link_frame[] = "base_link";
static char imu_frame[] = "imu_link";

// ---------- State Variables ----------
volatile uint32_t last_cmd_ms = 0;
float max_linear_vel = 1.0;   // unused now: cmd_vel is in real units, the dashboard scales speed itself
float current_conveyor_cmd = 0.0;

bool color_active = false;
bool imu_active = false;
bool imu_ok = false;          // true only while the IMU delivered a fresh sample this cycle; odometry keeps running without it

// True once the micro-ROS agent time sync has succeeded for the current session. Stamped topics are held back until then
// (an unsynced epoch clock gives bogus stamps and TF would drop every message).
bool time_synced = false;

// ---------- Yaw estimate ----------
float filtered_yaw = 0.0f;  // rad, EKF output
uint32_t last_filter_us = 0;

// ---------- Heading hold ----------
// Heading hold output is now a yaw rate in rad/s (cmd_ang is in rad/s): KP in 1/s, KD dimensionless, MAX in rad/s.
// These were tuned back when the command was normalised, so re-tune on the floor if the hold oscillates or is lazy.
#define HOLD_KP   1.5f
#define HOLD_KD   0.15f
#define HOLD_MAX  0.3f
float cmd_lin = 0.0f, cmd_ang = 0.0f;
float heading_target = 0.0f;
float hold_corr = 0.0f;
bool hold_active = false;
bool hold_enabled = true;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// =========================================================================================
// DUAL-CORE SPLIT (why the robot no longer stalls when the Pi / network lags)
//   core 1 (Arduino loop):  IMU sampling, wheel PI loops, EKF, heading hold, command watchdog. NEVER touches micro-ROS.
//   core 0 (uros_task):     agent ping / connect, executor spin, time sync, ALL publishing. May block as long as it likes.
// They only exchange data through two tiny structs guarded by a spinlock:
//   snap    control -> comms   latest sample wins; if comms is slow, old samples are overwritten, never queued
//   cmdbox  comms -> control   subscription callbacks only store the command; the control core applies it
// A stalled serial link / agent / Pi therefore cannot delay the motor control, the odometry or the CMD_TIMEOUT stop.
// =========================================================================================
#include "esp_timer.h"
static portMUX_TYPE ipc_mux = portMUX_INITIALIZER_UNLOCKED;

struct Snapshot {
  bool enc_new = false, imu_new = false, odom_new = false, color_new = false;
  int64_t stamp_us = 0;                         // esp_timer_get_time() when the sample was taken (converted to epoch by the comms core)
  int32_t ticks[4] = {0, 0, 0, 0};              // FL, RL, FR, RR
  float lax = 0, lay = 0, laz = 0, avx = 0, avy = 0, avz = 0;
  float qw = 1, qx = 0, qy = 0, qz = 0;         // IMU orientation (roll/pitch from the attitude filter, yaw from the EKF)
  float ox = 0, oy = 0, oqz = 0, oqw = 1, ovx = 0, owz = 0;   // odometry pose / velocity
  float p00 = 0, p01 = 0, p11 = 0, p22 = 0, p33 = 0;          // covariance entries
  float cr = 0, cg = 0, cb = 0;                 // colour sensor
};
static Snapshot snap;

struct CmdBox {
  bool  twist_new = false, pan_new = false, tilt_new = false, dump_new = false, conv_new = false, hold_new = false;
  float lin_x = 0, ang_z = 0; bool hold_req = false;
  float pan = 0, tilt = 90, dump = 15, conv = 0; bool hold_en = true;
};
static CmdBox cmdbox;
static volatile bool sc_publish_req = false;   // set by the control core, serviced by the comms core

// ---------- Motor Control ----------
#define MOTOR_MIN_PWM 150
#define MOTOR_MAX_PWM 250

void set_motor(int in1, int in2, float norm_speed) {
  norm_speed = constrain(norm_speed, -1.0f, 1.0f);
  int pwm = 0;
  if (fabs(norm_speed) > 0.01f) {
    pwm = MOTOR_MIN_PWM + (int)(fabs(norm_speed) * (MOTOR_MAX_PWM - MOTOR_MIN_PWM));
  }
  if (pwm == 0) {
    digitalWrite(in2, LOW); analogWrite(in1, 0);
  } else if (norm_speed > 0) {
    digitalWrite(in2, LOW); analogWrite(in1, pwm);
  } else {
    digitalWrite(in2, HIGH); analogWrite(in1, 255 - pwm);
  }
}

// Raw signed PWM (-255..255) to one H-bridge. Same pin logic as set_motor().
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

// One PI speed loop per wheel: u = F*r + sgn(r)*B + Kp*e + I,  I += Ki*dt*e   (gains from tuned_gains.json).
// All speeds are in the robot-forward-positive domain; MOTOR_DIR_* is applied only at the PWM pins.
struct WheelCtrl {
  const MotorGains* g;
  ESP32Encoder* enc;
  int enc_sign;
  float dir;            // MOTOR_DIR_* (+1/-1), may be int or float in pins.h
  int in1, in2;
  int32_t last_count;
  float target_rpm, integ, meas_rpm;
  int8_t last_sign;     // direction of the last target; integrator is cleared when it flips
};

WheelCtrl wheels[4] = {   // order: FL, RL, FR, RR
  {&GAINS_FL, &enc_fl, S_FL, MOTOR_DIR_FL, FL_IN1, FL_IN2, 0, 0, 0, 0},
  {&GAINS_RL, &enc_rl, S_RL, MOTOR_DIR_RL, RL_IN1, RL_IN2, 0, 0, 0, 0},
  {&GAINS_FR, &enc_fr, S_FR, MOTOR_DIR_FR, FR_IN1, FR_IN2, 0, 0, 0, 0},
  {&GAINS_RR, &enc_rr, S_RR, MOTOR_DIR_RR, RR_IN1, RR_IN2, 0, 0, 0, 0},
};

static void speed_ctrl_reset() {          // re-sync encoder baselines and clear integrators
  for (auto &w : wheels) {
    w.integ = 0.0f; w.meas_rpm = 0.0f; w.last_sign = 0;
    w.last_count = (int32_t)w.enc->getCount();
  }
}

static void speed_ctrl_stop() {           // immediate stop, zero targets
  for (auto &w : wheels) {
    w.target_rpm = 0.0f; w.integ = 0.0f;
    set_motor_pwm(w.in1, w.in2, 0);
  }
}

static void speed_ctrl_step(float dt) {
  for (auto &w : wheels) {
    int32_t c = (int32_t)w.enc->getCount();
    // Encoder sign = MOTOR_DIR, exactly as in PID_TEST.cpp (where these gains were tuned): flipping a motor's
    // polarity then flips its encoder reading too, so the feedback sign can never be wrong. (S_* is for odometry only.)
    float rpm = (float)(w.dir * (c - w.last_count)) / TICKS_PER_REV * 60.0f / dt;
    w.last_count = c;
    w.meas_rpm = rpm;

    float r = w.target_rpm;
    if (fabsf(r) < 0.5f) { w.integ = 0.0f; set_motor_pwm(w.in1, w.in2, 0); continue; }

    const Gain& g = (r > 0.0f) ? w.g->fwd : w.g->rev;     // separate gains per direction
    int8_t sgn = (r > 0.0f) ? 1 : -1;
    if (sgn != w.last_sign) { w.integ = 0.0f; w.last_sign = sgn; }

    float e  = r - rpm;
    float ff = g.ff * r + sgn * g.off;
    float u_pre = ff + g.kp * e + w.integ;
    bool saturated = fabsf(u_pre) >= MOTOR_MAX_PWM;
    if (!(saturated && e * u_pre > 0.0f)) w.integ += g.ki * dt * e;   // anti-windup: no integration into saturation
    w.integ = constrain(w.integ, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);

    float u = constrain(ff + g.kp * e + w.integ, -(float)MOTOR_MAX_PWM, (float)MOTOR_MAX_PWM);
    if (u * r < 0.0f) u = 0.0f;           // never drive against the commanded direction
    set_motor_pwm(w.in1, w.in2, (int)lroundf(u * w.dir));
  }
}

// Left/right normalised speed (-1..1 of MAX_WHEEL_SPEED) -> wheel rpm targets (executed by speed_ctrl_step()).
void drive_motors(float left_speed, float right_speed) {
  float left_norm  = constrain(left_speed,  -1.0f, 1.0f);
  float right_norm = constrain(right_speed, -1.0f, 1.0f);
  wheels[0].target_rpm = left_norm  * MAX_WHEEL_RPM;   // FL
  wheels[1].target_rpm = left_norm  * MAX_WHEEL_RPM;   // RL
  wheels[2].target_rpm = right_norm * MAX_WHEEL_RPM;   // FR
  wheels[3].target_rpm = right_norm * MAX_WHEEL_RPM;   // RR
  if (fabsf(left_norm) < 0.01f && fabsf(right_norm) < 0.01f) speed_ctrl_stop();
}

// ---------- ROS 2 Callbacks ----------
static void apply_drive() {
  float w  = cmd_ang + hold_corr - STRAIGHT_TRIM * cmd_lin;   // rad/s (trim cancels the robot's natural left veer)
  float vl = cmd_lin - w * TRACK_CMD * 0.5f;      // m/s
  float vr = cmd_lin + w * TRACK_CMD * 0.5f;
  float m  = fmaxf(fabsf(vl), fabsf(vr));
  if (m > MAX_WHEEL_SPEED) { float k = MAX_WHEEL_SPEED / m; vl *= k; vr *= k; }   // clip both sides, keep curvature
  drive_motors(vl / MAX_WHEEL_SPEED, vr / MAX_WHEEL_SPEED);
}

// Applies one command on the control core (the original twist_callback body).
static void handle_twist(float lin_x, float ang_z, bool hold_req) {
  last_cmd_ms = millis();
  cmd_ang = constrain(ang_z, -MAX_YAW_RATE, MAX_YAW_RATE);    // rad/s, capped
  cmd_lin = lin_x * (1.0f - TURN_SLOWDOWN * fabsf(cmd_ang) / MAX_YAW_RATE);   // m/s (TURN_SLOWDOWN = 0: unchanged)

  // Heading hold only when the sender asks for it (dashboard sets angular.x = 1, Nav2 leaves it 0)
  bool want_hold = hold_enabled && hold_req && imu_ok && fabsf(cmd_ang) < 0.02f && fabsf(cmd_lin) > 0.02f;
  if (want_hold && !hold_active) heading_target = filtered_yaw;
  if (!want_hold) hold_corr = 0.0f;
  hold_active = want_hold;

  apply_drive();
}

// Control core: pick up whatever the callbacks stored and apply it.
static void process_commands() {
  CmdBox c;
  portENTER_CRITICAL(&ipc_mux);
  c = cmdbox;
  cmdbox.twist_new = cmdbox.pan_new = cmdbox.tilt_new = cmdbox.dump_new = cmdbox.conv_new = cmdbox.hold_new = false;
  portEXIT_CRITICAL(&ipc_mux);

  if (c.hold_new) { hold_enabled = c.hold_en; if (!hold_enabled) { hold_active = false; hold_corr = 0.0f; } }
  if (c.pan_new)  pan_set_speed(c.pan);   // -1..1 speed, not an angle
  if (c.tilt_new) servo_write(CH_TILT, (int)constrain(c.tilt, TILT_MIN, TILT_MAX));
  if (c.dump_new) servo_write(CH_DUMP, constrain((int)c.dump, 15, 165));
  if (c.conv_new) { current_conveyor_cmd = c.conv; conveyorStepper.setSpeed(current_conveyor_cmd); }
  if (c.twist_new) handle_twist(c.lin_x, c.ang_z, c.hold_req);
}

// ---- subscription callbacks: run on the micro-ROS core, only store the command (no motor / servo / I2C access) ----
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  portENTER_CRITICAL(&ipc_mux);
  cmdbox.lin_x = msg->linear.x; cmdbox.ang_z = msg->angular.z; cmdbox.hold_req = msg->angular.x > 0.5f;
  cmdbox.twist_new = true;
  portEXIT_CRITICAL(&ipc_mux);
}

void pan_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  portENTER_CRITICAL(&ipc_mux); cmdbox.pan = msg->data; cmdbox.pan_new = true; portEXIT_CRITICAL(&ipc_mux);
}

void tilt_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  portENTER_CRITICAL(&ipc_mux); cmdbox.tilt = msg->y; cmdbox.tilt_new = true; portEXIT_CRITICAL(&ipc_mux);
}

void dump_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  portENTER_CRITICAL(&ipc_mux); cmdbox.dump = msg->data; cmdbox.dump_new = true; portEXIT_CRITICAL(&ipc_mux);
}

void conveyor_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  portENTER_CRITICAL(&ipc_mux); cmdbox.conv = msg->data; cmdbox.conv_new = true; portEXIT_CRITICAL(&ipc_mux);
}

void hold_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  portENTER_CRITICAL(&ipc_mux); cmdbox.hold_en = msg->data; cmdbox.hold_new = true; portEXIT_CRITICAL(&ipc_mux);
}

// ---------- I2C helpers ----------
static bool tcs_read(uint16_t &r, uint16_t &g, uint16_t &b) {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(0xB4);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(TCS_ADDR, 8, true) != 8) return false;
  uint8_t d[8];
  for (int i = 0; i < 8; i++) d[i] = Wire.read();
  r = (uint16_t)(d[2] | (d[3] << 8));
  g = (uint16_t)(d[4] | (d[5] << 8));
  b = (uint16_t)(d[6] | (d[7] << 8));
  return true;
}

// ---------- Self-check ----------
// BLOCKING: takes several seconds (motors, servos, stepper) with no spinning or publishing. Only trigger it while the robot is idle.
enum { SC_NOT_RUN = 0, SC_PASS = 1, SC_FAIL = 2, SC_UNVERIFIED = 3 };
enum { SC_MPU, SC_TCS, SC_MOT_FL, SC_MOT_RL, SC_MOT_FR, SC_MOT_RR,
       SC_TILT, SC_DUMP, SC_STEPPER, SC_PAN, SC_COUNT };   // SC_PAN appended last so the existing bit positions do not move
#define SC_MIN_COUNTS  5
#define SC_TEST_SPEED  0.3f

uint8_t sc_state[SC_COUNT] = {0};
bool sc_toggle = false;
bool sc_enabled = false;
volatile bool run_selfcheck = false;
rcl_publisher_t sc_pub;
std_msgs__msg__Int32 sc_msg;
rcl_subscription_t sc_sub;
std_msgs__msg__Int32 sc_cmd_msg;

static void selfcheck_publish() {
  if (!sc_enabled) return;
  int32_t v = 0;
  for (int i = 0; i < SC_COUNT; i++) v |= ((int32_t)(sc_state[i] & 3)) << (2 * i);
  if (sc_toggle) v |= (1 << 30);
  sc_msg.data = v;
  RCSOFTCHECK(rcl_publish(&sc_pub, &sc_msg, NULL));
}

void selfcheck_callback(const void * msgin) {
  (void)msgin;
  if (hold_active || fabsf(cmd_lin) > 0.01f || fabsf(cmd_ang) > 0.01f) return;
  run_selfcheck = true;
}

static uint8_t check_tcs() {
  if (!color_active) return SC_FAIL;
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(0x92);
  if (Wire.endTransmission(false) != 0) return SC_FAIL;
  if (Wire.requestFrom(TCS_ADDR, 1, true) != 1) return SC_FAIL;
  uint8_t id = Wire.read();
  return (id == 0x44 || id == 0x4D || id == 0x10) ? SC_PASS : SC_FAIL;
}

static uint8_t test_motor(int in1, int in2, ESP32Encoder &enc) {
  int64_t c0 = enc.getCount();
  set_motor(in1, in2, SC_TEST_SPEED);  delay(300);
  set_motor(in1, in2, 0.0f);           delay(150);
  int64_t c1 = enc.getCount();
  set_motor(in1, in2, -SC_TEST_SPEED); delay(300);
  set_motor(in1, in2, 0.0f);           delay(150);
  int64_t c2 = enc.getCount();

  int64_t f = c1 - c0, r = c2 - c1;
  int64_t af = f < 0 ? -f : f, ar = r < 0 ? -r : r;
  bool fok = af >= SC_MIN_COUNTS, rok = ar >= SC_MIN_COUNTS;
  bool dir = (f > 0) != (r > 0);
  enc.setCount(c0);   // restore the count so the test does not disturb odometry
  if (fok && rok && dir) return SC_PASS;
  if (!fok)              return SC_FAIL;
  if (!rok)              return SC_UNVERIFIED;
  return SC_NOT_RUN;                       // "Not run": both legs count, same sign
}

static void exercise_actuators() {
  servo_write(CH_TILT, (int)constrain(75, TILT_MIN, TILT_MAX));  delay(300);
  servo_write(CH_TILT, (int)constrain(105, TILT_MIN, TILT_MAX)); delay(300);
  servo_write(CH_TILT, 90);                                      delay(300);
  sc_state[SC_TILT] = pca_ok ? SC_UNVERIFIED : SC_FAIL;   // a PCA9685 that stopped answering is a real fail, not "check by eye"

  servo_write(CH_DUMP, 25); delay(500);
  servo_write(CH_DUMP, 15); delay(500);
  sc_state[SC_DUMP] = pca_ok ? SC_UNVERIFIED : SC_FAIL;

  pan_set_speed(0.5f);  delay(300);
  pan_set_speed(-0.5f); delay(300);
  pan_set_speed(0.0f);
  sc_state[SC_PAN] = pca_ok ? SC_UNVERIFIED : SC_FAIL;

  conveyorStepper.enableOutputs();
  conveyorStepper.setSpeed(300);
  uint32_t t0 = millis();
  while (millis() - t0 < 600) conveyorStepper.runSpeed();
  conveyorStepper.setSpeed(current_conveyor_cmd);
  conveyorStepper.disableOutputs();
  sc_state[SC_STEPPER] = SC_UNVERIFIED;
}

static void selfcheck_run() {
  cmd_lin = 0.0f; cmd_ang = 0.0f;
  hold_active = false; hold_corr = 0.0f;
  drive_motors(0, 0);

  sc_state[SC_MPU] = imuSensor.healthCheck() ? SC_PASS : SC_FAIL;
  sc_state[SC_TCS] = check_tcs();
  sc_state[SC_MOT_FL] = test_motor(FL_IN1, FL_IN2, enc_fl);
  sc_state[SC_MOT_RL] = test_motor(RL_IN1, RL_IN2, enc_rl);
  sc_state[SC_MOT_FR] = test_motor(FR_IN1, FR_IN2, enc_fr);
  sc_state[SC_MOT_RR] = test_motor(RR_IN1, RR_IN2, enc_rr);
  exercise_actuators();

  speed_ctrl_reset();   // self-check moved the wheels open loop (and rewrote encoder counts)
  last_filter_us = micros();
  sc_toggle = !sc_toggle;
  sc_publish_req = true;      // the comms core publishes it (the control core never calls rcl_*)
}

// cmd_vel_raw uses best-effort QoS (a reliable queue would replay old commands after a Pi/network stall);
// the CMD_TIMEOUT_MS watchdog in loop() stops the robot if commands stop arriving.
// ---------- micro-ROS connection management ----------
// Serial link budget: odometry+imu+encoders+color at 20/20/20/10 Hz need about 24 kB/s. 115200 baud carries only
// 11.5 kB/s, so the link saturates and a ping times out -> the session is dropped and re-created ("random restarts").
// MICROROS_BAUD must equal the agent's --baudrate (agent: serial --dev /dev/ttyUSBx -b 460800).
#define MICROROS_BAUD      460800
#define IMU_ODOM_PUB_DIV   1      // publish imu+odom every N-th 50 ms cycle (1 = 20 Hz). Use >= 4 if you must stay at 115200.
#define PING_PERIOD_MS     500    // agent liveness check interval
#define PING_TIMEOUT_MS    200    // per-ping timeout
#define PING_MAX_FAILS     4      // consecutive failed pings before the session is dropped (about 3 s)
#define TIME_RETRY_MS      1000   // retry the agent time sync this often until it succeeds
#define TIME_RESYNC_MS     300000UL // periodic re-sync once running
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;
bool hold_sub_ok = false;
#define RC_OK(fn) { if ((fn) != RCL_RET_OK) return false; }

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RC_OK(rclc_support_init(&support, 0, NULL, &allocator));
  RC_OK(rclc_node_init_default(&node, "base_controller", "", &support));

  RC_OK(rclc_publisher_init_best_effort(&enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "sensors/wheel_ticks"));
  RC_OK(rclc_publisher_init_best_effort(&color_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "sensors/color/rgb"));
  RC_OK(rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_data"));
  RC_OK(rclc_publisher_init_best_effort(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odometry/filtered"));

  // Stamped topics stay silent until this succeeds; loop() retries every TIME_RETRY_MS.
  rmw_uros_sync_session(1000);
  time_synced = rmw_uros_epoch_synchronized();

  RC_OK(rclc_subscription_init_best_effort(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_raw"));
  RC_OK(rclc_subscription_init_default(&pan_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "actuators/pan_cmd"));
  RC_OK(rclc_subscription_init_default(&tilt_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "actuators/tilt_cmd"));
  RC_OK(rclc_subscription_init_default(&dump_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "actuators/dump_cmd"));
  RC_OK(rclc_subscription_init_default(&conv_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "actuators/conveyor_cmd"));
  

  RC_OK(rclc_executor_init(&executor, &support.context, 7, &allocator));   // twist, pan, tilt, dump, conveyor, self-check, hold
  RC_OK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &pan_sub, &pan_msg, &pan_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &tilt_sub, &tilt_msg, &tilt_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &dump_sub, &dump_msg, &dump_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &conv_sub, &conv_msg, &conveyor_callback, ON_NEW_DATA));
  

  sc_enabled =
    rclc_publisher_init_best_effort(&sc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "self_check") == RCL_RET_OK &&
    rclc_subscription_init_default(&sc_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "self_check_cmd") == RCL_RET_OK &&
    rclc_executor_add_subscription(&executor, &sc_sub, &sc_cmd_msg, &selfcheck_callback, ON_NEW_DATA) == RCL_RET_OK;

  hold_sub_ok =
    rclc_subscription_init_default(&hold_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "heading_hold_cmd") == RCL_RET_OK &&
    rclc_executor_add_subscription(&executor, &hold_sub, &hold_msg, &hold_callback, ON_NEW_DATA) == RCL_RET_OK;
  return true;
}

static void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void)rcl_publisher_fini(&enc_pub, &node);
  (void)rcl_publisher_fini(&color_pub, &node);
  (void)rcl_publisher_fini(&imu_pub, &node);
  (void)rcl_publisher_fini(&odom_pub, &node);
  (void)rcl_publisher_fini(&sc_pub, &node);
  (void)rcl_subscription_fini(&twist_sub, &node);
  (void)rcl_subscription_fini(&pan_sub, &node);
  (void)rcl_subscription_fini(&tilt_sub, &node);
  (void)rcl_subscription_fini(&dump_sub, &node);
  (void)rcl_subscription_fini(&conv_sub, &node);
  (void)rcl_subscription_fini(&sc_sub, &node);
  (void)rcl_subscription_fini(&hold_sub, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);

  sc_enabled = false;
  hold_sub_ok = false;
  portENTER_CRITICAL(&ipc_mux); cmdbox.hold_en = true; cmdbox.hold_new = true; portEXIT_CRITICAL(&ipc_mux);   // heading hold is on again after every reconnect
  time_synced = false;
}

// ---------- Comms core: snapshot -> ROS messages ----------
static void publish_snapshot() {
  Snapshot sn;
  portENTER_CRITICAL(&ipc_mux);
  sn = snap;
  snap.enc_new = snap.imu_new = snap.odom_new = snap.color_new = false;
  portEXIT_CRITICAL(&ipc_mux);

  if (sn.color_new) {
    color_msg.x = sn.cr; color_msg.y = sn.cg; color_msg.z = sn.cb;
    RCSOFTCHECK(rcl_publish(&color_pub, &color_msg, NULL));
  }
  if (!(sn.enc_new || sn.imu_new || sn.odom_new) || !time_synced) return;   // stamped topics wait for the time sync

  int64_t age_us = esp_timer_get_time() - sn.stamp_us;
  if (age_us > 500000) return;                                   // stale (long stall): drop it instead of publishing old data
  int64_t t_ns = rmw_uros_epoch_nanos() - age_us * 1000LL;       // stamp = time the sample was TAKEN, not when it is sent
  int32_t sec = (int32_t)(t_ns / 1000000000LL);
  uint32_t nsec = (uint32_t)(t_ns % 1000000000LL);

  if (sn.enc_new) {
    for (int i = 0; i < 4; i++) enc_msg.data.data[i] = sn.ticks[i];
    enc_msg.data.data[4] = sec;
    enc_msg.data.data[5] = (int32_t)nsec;
    RCSOFTCHECK(rcl_publish(&enc_pub, &enc_msg, NULL));
  }

  if (sn.imu_new) {
    imu_msg.header.stamp.sec = sec; imu_msg.header.stamp.nanosec = nsec;
    imu_msg.linear_acceleration.x = sn.lax; imu_msg.linear_acceleration.y = sn.lay; imu_msg.linear_acceleration.z = sn.laz;
    imu_msg.angular_velocity.x = sn.avx; imu_msg.angular_velocity.y = sn.avy; imu_msg.angular_velocity.z = sn.avz;
    imu_msg.orientation.w = sn.qw; imu_msg.orientation.x = sn.qx; imu_msg.orientation.y = sn.qy; imu_msg.orientation.z = sn.qz;
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));          // only flagged while the IMU delivered fresh data
  }

  if (sn.odom_new) {
    odom_msg.header.stamp.sec = sec; odom_msg.header.stamp.nanosec = nsec;
    odom_msg.pose.pose.position.x = sn.ox;
    odom_msg.pose.pose.position.y = sn.oy;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;                      // yaw-only orientation for the 2D odometry
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sn.oqz;
    odom_msg.pose.pose.orientation.w = sn.oqw;
    odom_msg.twist.twist.linear.x = sn.ovx;                      // fused EKF velocity
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = sn.owz;

    // Pose covariance (row-major 6x6: x=0, y=7, yaw=35). Unobserved axes get a large value.
    for (int i = 0; i < 36; i++) { odom_msg.pose.covariance[i] = 0.0; odom_msg.twist.covariance[i] = 0.0; }
    odom_msg.pose.covariance[0]  = sn.p00; odom_msg.pose.covariance[1]  = sn.p01;
    odom_msg.pose.covariance[6]  = sn.p01; odom_msg.pose.covariance[7]  = sn.p11;
    odom_msg.pose.covariance[35] = sn.p22;
    odom_msg.pose.covariance[14] = odom_msg.pose.covariance[21] = odom_msg.pose.covariance[28] = 1e6;
    odom_msg.twist.covariance[0]  = sn.p33;                      // variance of linear velocity
    odom_msg.twist.covariance[35] = 0.05;                        // wz, PLACEHOLDER
    odom_msg.twist.covariance[7]  = odom_msg.twist.covariance[14] = odom_msg.twist.covariance[21] = odom_msg.twist.covariance[28] = 1e6;
    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
  }
}

// Everything that can block on the Pi / serial link lives here, on core 0.
static void uros_task(void *arg) {
  (void)arg;
  uint32_t last_agent_check = 0, last_sync_ms = 0, last_sc_pub = 0;
  uint8_t ping_fail = 0;
  for (;;) {
    uint32_t now = millis();
    switch (agent_state) {
      case WAITING_AGENT:
        if (now - last_agent_check >= 500) {
          last_agent_check = now;
          agent_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
        }
        break;
      case AGENT_AVAILABLE:
        if (create_entities()) {
          ping_fail = 0;
          last_agent_check = millis();
          agent_state = AGENT_CONNECTED;
        } else {
          destroy_entities();
          agent_state = WAITING_AGENT;
        }
        break;
      case AGENT_CONNECTED:
        if (now - last_agent_check >= PING_PERIOD_MS) {
          last_agent_check = now;
          if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
            ping_fail = 0;
          } else if (++ping_fail >= PING_MAX_FAILS) {          // tolerate a few missed pings on a busy link
            ping_fail = 0;
            agent_state = AGENT_DISCONNECTED;
          }
        }
        if (agent_state == AGENT_CONNECTED) {
          RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
        }
        break;
      case AGENT_DISCONNECTED:
        destroy_entities();
        agent_state = WAITING_AGENT;
        break;
    }

    const bool connected = (agent_state == AGENT_CONNECTED);
    now = millis();

    // Time sync: retry until it succeeds, then re-sync periodically. Stamped topics wait for time_synced.
    if (connected && !time_synced) {
      if (now - last_sync_ms >= TIME_RETRY_MS) {
        last_sync_ms = now;
        rmw_uros_sync_session(200);
        time_synced = rmw_uros_epoch_synchronized();
      }
    } else if (connected && now - last_sync_ms >= TIME_RESYNC_MS) {
      last_sync_ms = now;
      rmw_uros_sync_session(200);
      time_synced = rmw_uros_epoch_synchronized();
    }

    if (connected) {
      publish_snapshot();
      if (sc_enabled && (sc_publish_req || now - last_sc_pub >= 1000)) {
        sc_publish_req = false;
        last_sc_pub = now;
        selfcheck_publish();
      }
    }
    vTaskDelay(1);   // 1 ms: feeds the core-0 idle watchdog and keeps the executor responsive
  }
}

void setup() {
  Serial.begin(MICROROS_BAUD);
  set_microros_serial_transports(Serial);
  analogWriteFrequency(20000);

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  drive_motors(0, 0);
  conveyorStepper.setMaxSpeed(700.0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B); enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B); enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);
  enc_msg.data.capacity = 6; enc_msg.data.size = 6; enc_msg.data.data = enc_data;

  Wire.begin(I2C_SDA, I2C_SCL);
  pca_ok = pca_init();                                        // before the multi-second IMU calibration below
  pan_set_speed(0.0f); servo_write(CH_TILT, 90); servo_write(CH_DUMP, 15);   // pan stopped, tilt/dump at their start positions
  if (tcs.begin()) color_active = true;

  if (imuSensor.begin()) {
    imu_active = true;
    imuSensor.calibrateGyro();   // robot must be still (~1.2 s)
    gz_bias0 = imuSensor.getGyroBiasZ();
    attitudeCalibrate();         // robot must be still and level (~2 s)
    last_filter_us = micros();
  }

  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame);

  imu_msg.orientation_covariance[0] = 0.01;   // roll: from attitude filter
  imu_msg.orientation_covariance[4] = 0.01;   // pitch: from attitude filter
  imu_msg.orientation_covariance[8] = 1e3;    // yaw: gyro-integrated, not an absolute heading, so tell consumers not to trust it
  imu_msg.angular_velocity_covariance[0] = 0.001;
  imu_msg.angular_velocity_covariance[4] = 0.001;
  imu_msg.angular_velocity_covariance[8] = 0.001;
  imu_msg.linear_acceleration_covariance[0] = 0.05;
  imu_msg.linear_acceleration_covariance[4] = 0.05;
  imu_msg.linear_acceleration_covariance[8] = 0.05;

  odom_msg.header.frame_id.data = odom_frame;
  odom_msg.header.frame_id.size = strlen(odom_frame);
  odom_msg.header.frame_id.capacity = sizeof(odom_frame);

  odom_msg.child_frame_id.data = base_link_frame;
  odom_msg.child_frame_id.size = strlen(base_link_frame);
  odom_msg.child_frame_id.capacity = sizeof(base_link_frame);

  agent_state = WAITING_AGENT;
  xTaskCreatePinnedToCore(uros_task, "uros", 20480, NULL, 2, NULL, 0);   // stack in bytes; raise it if create_entities() overflows
}

void loop() {
  process_commands();                 // apply whatever the comms core received (never blocks)
  uint32_t current_time = millis();

  // 200 Hz IMU sampling; averaged in the 50 ms block below
  static uint32_t last_fast_us = 0;
  if (imu_active && (micros() - last_fast_us) >= 5000) {
    last_fast_us = micros();
    float v[6];
    if (imuSensor.readData(v[0], v[1], v[2], v[3], v[4], v[5])) {
      for (int j = 0; j < 6; j++) imu_acc[j] += v[j];
      imu_acc_n++;
    }
  }

  if (run_selfcheck) {
    run_selfcheck = false;
    selfcheck_run();
    current_time = millis();
  }
  static uint32_t last_speed_ms = 0;
  if (current_time - last_speed_ms >= SPEED_LOOP_MS) {
    float dt = (current_time - last_speed_ms) * 1e-3f;
    last_speed_ms = current_time;
    if (dt > 0.25f) speed_ctrl_reset();   // first tick after a long block (self-check): just re-sync
    else            speed_ctrl_step(dt);
  }

  if (abs(current_conveyor_cmd) > 0.1) {
    conveyorStepper.enableOutputs(); conveyorStepper.runSpeed();
  } else {
    conveyorStepper.disableOutputs();
  }

  static uint32_t last_imu_pub = 0;
  if (current_time - last_imu_pub >= 50) {
    last_imu_pub = current_time;

    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    imu_ok = imu_active && imu_acc_n > 0;
    if (imu_ok) {
      float inv = 1.0f / imu_acc_n;
      ax = imu_acc[0] * inv; ay = imu_acc[1] * inv; az = imu_acc[2] * inv;
      gx = imu_acc[3] * inv; gy = imu_acc[4] * inv; gz = imu_acc[5] * inv;
    }
    for (int j = 0; j < 6; j++) imu_acc[j] = 0.0f;
    imu_acc_n = 0;
    if (!imu_ok) { ax = ay = az = gx = gy = gz = 0.0f; }   // estimator keeps running on the encoders alone
    {
      // Sensor frame is standard FLU (x fwd, y left, z up): no axis flips.

      uint32_t now_us = micros();
      float dt = (now_us - last_filter_us) * 1e-6f;
      last_filter_us = now_us;

      if (dt > 0.0f && dt < 0.5f) {

        // Roll/pitch filter + ramp flag. Runs on the driver-corrected gz,
        // before ZUPT/deadband modify it.
        if (imu_ok) att.update(ax, ay, az, gx, gy, gz, dt);

        // --- Zero Velocity Update + gyro bias learning ---
        static uint32_t still_since = 0;
        bool still    = fabsf(cmd_lin) < 0.003f && fabsf(cmd_ang) < 0.01f;   // m/s, rad/s (lowest moving command is about 0.0017 m/s)
        bool straight = !still && fabsf(cmd_ang) < 0.01f;                    // driving, no turn commanded
        if (still) { if (still_since == 0) still_since = current_time; }
        else       { still_since = 0; }

        // Standing still: unclamped learning like the original, wide gate so a bad boot calibration can recover.
        // Driving straight: slow learning, gated tightly and clamped around the boot calibration.
        float k_b = 0.0f;
        bool  clamp_b = false;
        if (imu_ok) {
          if (still && current_time - still_since > 1000 && fabsf(gz) < 0.15f) { k_b = 0.02f; }
          else if (straight && fabsf(gz) < 0.05f)                              { k_b = 0.001f; clamp_b = true; }
        }
        if (k_b > 0.0f) {
          float b = imuSensor.getGyroBiasZ() + k_b * gz / imuSensor.getGyroZScale();   // bias is in raw (unscaled) units
          if (clamp_b) b = constrain(b, gz_bias0 - 0.01f, gz_bias0 + 0.01f);  // +-0.01 rad/s around the boot calibration
          imuSensor.updateGyroBiasZ(b);
          gz -= k_b * gz;                                                     // apply instantly to this tick
        }

        // Deadband only while fully stopped: while driving, small real yaw drift (veer) must reach the heading estimate
        if (still && fabsf(gz) < 0.005f) gz = 0.0f;

        // --- EKF Update Process (4-State) ---
        int32_t enc_l = (S_FL * enc_fl.getCount() + S_RL * enc_rl.getCount()) / 2;
        int32_t enc_r = (S_FR * enc_fr.getCount() + S_RR * enc_rr.getCount()) / 2;

        float d_left = (enc_l - last_enc_l) * M_PER_TICK;
        float d_right = (enc_r - last_enc_r) * M_PER_TICK;
        last_enc_l = enc_l;
        last_enc_r = enc_r;

        float d_center = (d_left + d_right) / 2.0f;
        float v_enc = d_center / dt;   // yaw from the encoders is deliberately not used (skid steer under-reads turns)

        // No IMU: yaw rate from the encoders using the effective track (skid steer under-reads with the geometric one), no accelerometer.
        if (!imu_ok) gz = (d_right - d_left) / TRACK_CMD / dt;
        // Gravity-compensated forward accel in m/s^2 (gain-corrected, absolute-pitch-compensated)
        float accel_forward = imu_ok ? att.forwardAccel() : 0.0f;
        // Odometry is planar: project the along-slope speed onto the horizontal plane
        float cph = imu_ok ? cosf(att.pitch) : 1.0f;

        // 1. Prediction Step
        float v_pred = X(3) + (accel_forward * dt);
        float theta_pred = X(2) + (gz * dt);

        // ZUPT: Kill accelerometer drift when commanded to stop
        if (still) {
            v_pred = 0.0f;
        }

        X(0) += v_pred * cph * cosf(theta_pred) * dt;
        X(1) += v_pred * cph * sinf(theta_pred) * dt;
        X(2) = theta_pred;
        X(3) = v_pred;

        // 2. Jacobian Matrix F
        BLA::Matrix<4, 4> F = {
            1.0f, 0.0f, -v_pred * cph * sinf(theta_pred) * dt, cph * cosf(theta_pred) * dt,
            0.0f, 1.0f,  v_pred * cph * cosf(theta_pred) * dt, cph * sinf(theta_pred) * dt,
            0.0f, 0.0f,  1.0f,                           0.0f,
            0.0f, 0.0f,  0.0f,                           1.0f
        };

        P = F * P * ~F + Q;
        if (!imu_ok) P(2, 2) += 1e-4f;   // encoder yaw is much worse than the gyro: grow yaw covariance faster (my estimate, tune it)

        // 3. Measurement update: encoder forward velocity only, H = [0 0 0 1].
        // Scalar update, so no 4x4 inverse. x, y, theta covariance keeps growing between updates (honest dead reckoning).
        float y_v = v_enc - X(3);
        float S_v = P(3, 3) + R(3, 3);
        float k[4] = {P(0, 3) / S_v, P(1, 3) / S_v, P(2, 3) / S_v, P(3, 3) / S_v};
        for (int i = 0; i < 4; i++) X(i) += k[i] * y_v;
        BLA::Matrix<4, 4> Pn = P;
        for (int i = 0; i < 4; i++)
          for (int j = 0; j < 4; j++) Pn(i, j) = P(i, j) - k[i] * P(3, j);
        P = Pn;

        while (X(2) > PI)  X(2) -= 2.0f * PI;
        while (X(2) < -PI) X(2) += 2.0f * PI;

        filtered_yaw = X(2);

        // --- Heading Hold Control ---
        if (!imu_ok && hold_active) { hold_active = false; hold_corr = 0.0f; apply_drive(); }   // no gyro, no heading hold
        if (hold_active && (current_time - last_cmd_ms <= CMD_TIMEOUT_MS)) {
          float err = heading_target - filtered_yaw;
          while (err >  PI) err -= 2.0f * PI;
          while (err < -PI) err += 2.0f * PI;
          hold_corr = constrain(HOLD_KP * err - HOLD_KD * gz, -HOLD_MAX, HOLD_MAX);
          apply_drive();
        }

        // --- Hand the result to the comms core (latest wins; nothing here can block on the link) ---
        float cr = cosf(att.roll * 0.5f),  sr = sinf(att.roll * 0.5f);
        float cp = cosf(att.pitch * 0.5f), sp = sinf(att.pitch * 0.5f);
        float cy = cosf(filtered_yaw * 0.5f), sy = sinf(filtered_yaw * 0.5f);
        int32_t t0 = (int32_t)enc_fl.getCount(), t1 = (int32_t)enc_rl.getCount();
        int32_t t2 = (int32_t)enc_fr.getCount(), t3 = (int32_t)enc_rr.getCount();
        static uint8_t pub_cnt = 0;
        bool pub_now = (++pub_cnt >= IMU_ODOM_PUB_DIV);
        if (pub_now) pub_cnt = 0;

        portENTER_CRITICAL(&ipc_mux);
        snap.stamp_us = esp_timer_get_time();
        snap.ticks[0] = t0; snap.ticks[1] = t1; snap.ticks[2] = t2; snap.ticks[3] = t3;
        snap.enc_new = true;
        if (pub_now) {
          snap.lax = ax * att.gainK(); snap.lay = ay * att.gainK(); snap.laz = az * att.gainK();
          snap.avx = gx - att.gyroBias(0); snap.avy = gy - att.gyroBias(1); snap.avz = gz;
          // ZYX Euler -> quaternion: roll/pitch from the attitude filter, yaw from the EKF
          snap.qw = cr*cp*cy + sr*sp*sy;
          snap.qx = sr*cp*cy - cr*sp*sy;
          snap.qy = cr*sp*cy + sr*cp*sy;
          snap.qz = cr*cp*sy - sr*sp*cy;
          snap.ox = X(0); snap.oy = X(1); snap.oqz = sy; snap.oqw = cy;
          snap.ovx = X(3); snap.owz = gz;
          snap.p00 = P(0, 0); snap.p01 = P(0, 1); snap.p11 = P(1, 1); snap.p22 = P(2, 2); snap.p33 = P(3, 3);
          snap.imu_new = imu_ok;                // never publish stale IMU data
          snap.odom_new = true;
        }
        portEXIT_CRITICAL(&ipc_mux);
      } else {
        // dt out of range (long block / reconnect): re-baseline the encoder deltas so the next tick does not
        // see the whole accumulated count as one step (velocity spike -> position jump)
        last_enc_l = (S_FL * enc_fl.getCount() + S_RL * enc_rl.getCount()) / 2;
        last_enc_r = (S_FR * enc_fr.getCount() + S_RR * enc_rr.getCount()) / 2;
      }
    }
  }

  static uint32_t last_color_pub = 0;
  static bool color_primed = false;
  if (color_active && current_time - last_color_pub >= 100) {
    last_color_pub = current_time;
    uint16_t r, g, b;
    if (tcs_read(r, g, b)) {
      if (!color_primed) {
        color_primed = true;
      } else {
        portENTER_CRITICAL(&ipc_mux);
        snap.cr = r; snap.cg = g; snap.cb = b; snap.color_new = true;
        portEXIT_CRITICAL(&ipc_mux);
      }
    }
  }

  if (pan_active && current_time - pan_last_ms > CMD_TIMEOUT_MS) pan_set_speed(0.0f);   // pan watchdog: no command, no motion

  static bool is_stopped = false;
  if (current_time - last_cmd_ms > CMD_TIMEOUT_MS) {
    if (!is_stopped) {
      cmd_lin = 0.0f; cmd_ang = 0.0f;
      hold_active = false; hold_corr = 0.0f;
      drive_motors(0, 0);
      is_stopped = true;
    }
  } else {
    is_stopped = false;
  }
}