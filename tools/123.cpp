#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

#include <ESP32Servo.h>
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
//   sensors/wheel_ticks  std_msgs/Int32MultiArray  10 Hz  [FL, RL, FR, RR ticks, stamp_sec, stamp_nanosec]   (ENC_PUB_MS)
//   imu_data             sensor_msgs/Imu           10 Hz  frame imu_link; roll/pitch from attitude filter, yaw from gyro integration (IMU_PUB_DIV)
//                                                         (orientation yaw covariance is 1e3: do not fuse it as an absolute heading)
//   odometry/filtered    nav_msgs/Odometry         20 Hz  frame odom -> child base_link; EKF pose + velocity (ODOM_PUB_DIV)
//                                                         NO TF is broadcast by the ESP32: the RPi5 must publish odom -> base_link from this topic
//   sensors/color/rgb    geometry_msgs/Vector3     10 Hz  TCS34725 raw R, G, B
//   self_check           std_msgs/Int32             1 Hz  2 bits per item (0 not run, 1 pass, 2 fail, 3 check by eye), bit 30 = run toggle
// Subscribed by the ESP32 (default reliable QoS):
//   cmd_vel_raw          geometry_msgs/Twist       drive command in m/s and rad/s (from twist_mux); angular.x > 0.5 asks for heading hold
//   actuators/tilt_cmd   geometry_msgs/Point       y = tilt servo angle in degrees (x is ignored, there is no pan servo)
//   actuators/dump_cmd   std_msgs/Float32          dump servo angle, 15..165 degrees
//   actuators/conveyor_cmd std_msgs/Float32        conveyor stepper speed in steps/s (+-700)
//   self_check_cmd       std_msgs/Int32            any message starts the self-check (ignored while driving)
//   heading_hold_cmd     std_msgs/Bool             master switch for heading hold (true again after every agent reconnect)
// Timestamps (wheel_ticks, imu_data, odometry/filtered) are agent-synchronised epoch time. Nothing stamped is published
// until the time sync with the agent has succeeded, so the agent must run on the same machine/clock as the lidar driver.
// =========================================================================================

// ---------- Robot Kinematics (Tune these in meters) ----------
const float WHEEL_RADIUS = 0.0381f; // 76.2 mm wheel diameter (verified by distance test)
const float TRACK_WIDTH = 0.215f;
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
// Cap on the commanded turn rate. At 2 rad/s a 10 Hz lidar rotates about 11 deg per scan, which smears scans and breaks
// scan matching. 1.0 rad/s is a safer value for mapping; raise it again if Nav2 needs more (keep dashboard WMAX <= this).
const float MAX_YAW_RATE = 1.0f;   // rad/s
// Fraction of linear speed removed at full turn rate. 0 = pass cmd_vel through unchanged (right for Nav2).
#define TURN_SLOWDOWN 0.0f

// Wheel labels are from the robot's own perspective (standing behind it, looking the way it drives).
// Encoder sign for odometry (counts positive when the wheel rolls forward). It must equal MOTOR_DIR_*.
const int S_FL = MOTOR_DIR_FL, S_RL = MOTOR_DIR_RL, S_FR = MOTOR_DIR_FR, S_RR = MOTOR_DIR_RR;

// ---------- Timing / load knobs ----------
#define I2C_HZ            400000   // 100 kHz made the 200 Hz IMU read cost ~30 % of the loop. Drop to 100000 if the bus gets flaky near the motors.
#define I2C_TIMEOUT_MS    10       // a hung I2C transfer must not stall the loop for the default 50 ms
#define IMU_SAMPLE_MS     5        // IMU task period (200 Hz)
#define EST_PERIOD_MS     50       // estimator tick
#define EST_DT_MAX        1.0f     // s; longer gaps re-baseline the encoders and skip the tick
#define ODOM_PUB_DIV      1        // publish odom every N-th estimator tick (1 = 20 Hz)
#define IMU_PUB_DIV       2        // publish imu every N-th tick (2 = 10 Hz). SLAM does not use it.
#define ENC_PUB_MS        100      // wheel_ticks period
#define SERIAL_RX_BUF     2048     // default 256 B overflows when the loop is busy at 460800 baud

// ---------- EKF Matrices (4-State) ----------
// State X = [x, y, theta, v]. Prediction: gyro for theta, gravity-compensated accelerometer for v.
// Measurement: encoder forward velocity only (H = [0 0 0 1]). x, y and theta are dead-reckoned.
BLA::Matrix<4> X = {0.0f, 0.0f, 0.0f, 0.0f}; // State vector [x, y, theta, v]
BLA::Matrix<4, 4> P = {1.0f, 0.0f, 0.0f, 0.0f,
                       0.0f, 1.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 1.0f, 0.0f,
                       0.0f, 0.0f, 0.0f, 1.0f}; // Covariance Matrix

// Process noise per 50 ms step. Q(2,2) = 1e-5 is an estimate for gyro-integrated yaw: tune it against measured yaw drift.
BLA::Matrix<4, 4> Q = {0.001f, 0.0f,   0.0f,   0.0f,
                       0.0f,   0.001f, 0.0f,   0.0f,
                       0.0f,   0.0f,   1e-5f,  0.0f,
                       0.0f,   0.0f,   0.0f,   0.05f}; // Process Noise (Gyro & Accel trust)

// Measurement noise: only R(3,3) (encoder velocity) is used.
BLA::Matrix<4, 4> R = {0.05f, 0.0f,  0.0f,  0.0f,
                       0.0f,  0.05f, 0.0f,  0.0f,
                       0.0f,  0.0f,  0.1f,  0.0f,
                       0.0f,  0.0f,  0.0f,  0.02f}; // Measurement Noise (Encoder trust)

int32_t last_enc_l = 0;
int32_t last_enc_r = 0;

// ---------- Hardware Instances ----------
Servo tilt_servo, dump_servo;
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;
AccelStepper conveyorStepper(AccelStepper::FULL4WIRE, CONV_IN1, CONV_IN3, CONV_IN2, CONV_IN4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
MPU6050_Custom imuSensor;
Attitude att;   // roll/pitch complementary filter + ramp flag (ROS FLU frame)

// ---------- I2C sharing: the IMU task and loop() (TCS34725, self-check) use the same bus ----------
static SemaphoreHandle_t i2c_mtx = NULL;
struct I2CLock {
  I2CLock()  { xSemaphoreTake(i2c_mtx, portMAX_DELAY); }
  ~I2CLock() { xSemaphoreGive(i2c_mtx); }
};

// ---------- IMU sampling task (200 Hz on core 0, independent of micro-ROS stalls in loop()) ----------
// The task sums samples; the 50 ms estimator tick takes the sums and averages them. A loop() stall therefore just
// gives a longer averaging window instead of an empty one.
static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static float    imu_acc[6] = {0, 0, 0, 0, 0, 0};   // ax ay az gx gy gz sums
static uint16_t imu_acc_n  = 0;

static void imu_task(void *) {
  const TickType_t period = max((TickType_t)1, (TickType_t)pdMS_TO_TICKS(IMU_SAMPLE_MS));
  TickType_t t = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&t, period);
    float v[6];
    bool ok;
    {
      I2CLock lock;
      ok = imuSensor.readData(v[0], v[1], v[2], v[3], v[4], v[5]);
    }
    if (ok) {
      portENTER_CRITICAL(&imu_mux);
      for (int j = 0; j < 6; j++) imu_acc[j] += v[j];
      imu_acc_n++;
      portEXIT_CRITICAL(&imu_mux);
    }
  }
}

static void take_imu_window(float out[6], uint16_t &n) {
  portENTER_CRITICAL(&imu_mux);
  n = imu_acc_n;
  for (int j = 0; j < 6; j++) { out[j] = imu_acc[j]; imu_acc[j] = 0.0f; }
  imu_acc_n = 0;
  portEXIT_CRITICAL(&imu_mux);
}

#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2400

const int TCS_ADDR = 0x29;

// Average ~2 s of still samples and calibrate the attitude filter (setup only, before the IMU task starts).
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
geometry_msgs__msg__Point pan_msg;
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
float current_conveyor_cmd = 0.0;

bool color_active = false;
bool imu_active = false;
bool imu_ok = false;          // true while the IMU delivered samples in the last estimator window

// True once the micro-ROS agent time sync has succeeded for the current session. Stamped topics are held back until then.
bool time_synced = false;
uint32_t last_sync_ms = 0;
uint32_t g_still_ms = 0;      // how long the robot has been commanded still (ms), 0 while moving

// ---------- Yaw estimate ----------
float filtered_yaw = 0.0f;  // rad, EKF output
uint32_t last_filter_us = 0;

// ---------- Heading hold ----------
#define HOLD_KP   1.5f
#define HOLD_KD   0.15f
#define HOLD_MAX  0.3f
float cmd_lin = 0.0f, cmd_ang = 0.0f;
float heading_target = 0.0f;
float hold_corr = 0.0f;
bool hold_active = false;
bool hold_enabled = true;

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

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
    if (!(saturated && e * u_pre > 0.0f)) w.integ += g.ki * dt * e;   // anti-windup
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
  float w  = cmd_ang + hold_corr;                 // rad/s
  float vl = cmd_lin - w * TRACK_CMD * 0.5f;      // m/s
  float vr = cmd_lin + w * TRACK_CMD * 0.5f;
  float m  = fmaxf(fabsf(vl), fabsf(vr));
  if (m > MAX_WHEEL_SPEED) { float k = MAX_WHEEL_SPEED / m; vl *= k; vr *= k; }   // clip both sides, keep curvature
  drive_motors(vl / MAX_WHEEL_SPEED, vr / MAX_WHEEL_SPEED);
}

void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_ms = millis();
  cmd_ang = constrain(msg->angular.z, -MAX_YAW_RATE, MAX_YAW_RATE);    // rad/s, capped
  cmd_lin = msg->linear.x * (1.0f - TURN_SLOWDOWN * fabsf(cmd_ang) / MAX_YAW_RATE);   // m/s

  // Heading hold only when the sender asks for it (dashboard sets angular.x = 1, Nav2 leaves it 0)
  bool hold_req  = msg->angular.x > 0.5f;
  bool want_hold = hold_enabled && hold_req && imu_ok && fabsf(cmd_ang) < 0.02f && fabsf(cmd_lin) > 0.02f;
  if (want_hold && !hold_active) heading_target = filtered_yaw;
  if (!want_hold) hold_corr = 0.0f;
  hold_active = want_hold;

  apply_drive();
}

void pan_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  tilt_servo.write((int)constrain(msg->y, TILT_MIN, TILT_MAX));
}

void dump_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  dump_servo.write(constrain((int)msg->data, 15, 165));
}

void conveyor_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  current_conveyor_cmd = msg->data;
  conveyorStepper.setSpeed(current_conveyor_cmd);
}

void hold_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  hold_enabled = msg->data;
  if (!hold_enabled) { hold_active = false; hold_corr = 0.0f; }
}

// ---------- I2C helpers (all take the bus lock) ----------
static bool tcs_read(uint16_t &r, uint16_t &g, uint16_t &b) {
  I2CLock lock;
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
       SC_TILT, SC_DUMP, SC_STEPPER, SC_COUNT };
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
  I2CLock lock;
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
  tilt_servo.write((int)constrain(75, TILT_MIN, TILT_MAX));  delay(300);
  tilt_servo.write((int)constrain(105, TILT_MIN, TILT_MAX)); delay(300);
  tilt_servo.write(90);                                      delay(300);
  sc_state[SC_TILT] = SC_UNVERIFIED;

  dump_servo.write(25); delay(500);
  dump_servo.write(15); delay(500);
  sc_state[SC_DUMP] = SC_UNVERIFIED;

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

  {
    I2CLock lock;
    sc_state[SC_MPU] = imuSensor.healthCheck() ? SC_PASS : SC_FAIL;
  }
  sc_state[SC_TCS] = check_tcs();
  sc_state[SC_MOT_FL] = test_motor(FL_IN1, FL_IN2, enc_fl);
  sc_state[SC_MOT_RL] = test_motor(RL_IN1, RL_IN2, enc_rl);
  sc_state[SC_MOT_FR] = test_motor(FR_IN1, FR_IN2, enc_fr);
  sc_state[SC_MOT_RR] = test_motor(RR_IN1, RR_IN2, enc_rr);
  exercise_actuators();

  speed_ctrl_reset();   // self-check moved the wheels open loop (and rewrote encoder counts)
  { float s[6]; uint16_t n; take_imu_window(s, n); }   // discard the multi-second window collected during the test
  last_filter_us = micros();
  sc_toggle = !sc_toggle;
  selfcheck_publish();
}

// ---------- micro-ROS connection management ----------
// Link budget: odometry carries two 36-double covariance arrays (about 750 B per message), so odom alone is about 15 kB/s
// at 20 Hz. 460800 baud carries 46 kB/s, 921600 carries 92 kB/s. Every byte is written in a blocking call from loop().
// MICROROS_BAUD must equal the agent's --baudrate (agent: serial --dev /dev/ttyUSBx -b 460800).
#define MICROROS_BAUD      460800
#define PING_PERIOD_MS     500    // agent liveness check interval
#define PING_TIMEOUT_MS    200    // per-ping timeout
#define PING_MAX_FAILS     4      // consecutive failed pings before the session is dropped
#define TIME_RETRY_MS      1000   // retry the agent time sync this often until it succeeds
#define TIME_RETRY_TIMEOUT 100    // ms blocked per retry
#define TIME_RESYNC_MS     300000UL // periodic re-sync once running (only done while the robot is still)
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
  last_sync_ms = millis();

  RC_OK(rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_raw"));
  RC_OK(rclc_subscription_init_default(&pan_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "actuators/tilt_cmd"));
  RC_OK(rclc_subscription_init_default(&dump_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "actuators/dump_cmd"));
  RC_OK(rclc_subscription_init_default(&conv_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "actuators/conveyor_cmd"));

  RC_OK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RC_OK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &pan_sub, &pan_msg, &pan_callback, ON_NEW_DATA));
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
  (void)rcl_subscription_fini(&dump_sub, &node);
  (void)rcl_subscription_fini(&conv_sub, &node);
  (void)rcl_subscription_fini(&sc_sub, &node);
  (void)rcl_subscription_fini(&hold_sub, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);

  sc_enabled = false;
  hold_sub_ok = false;
  hold_enabled = true;
  time_synced = false;
}

// ---------- Estimator tick (every EST_PERIOD_MS): gyro window -> EKF -> publish ----------
static void estimator_tick(uint32_t now_ms, bool connected) {
  // 1. Take the IMU window collected by imu_task
  float s[6]; uint16_t n;
  take_imu_window(s, n);
  imu_ok = imu_active && n > 0;
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (imu_ok) {
    float inv = 1.0f / n;
    ax = s[0] * inv; ay = s[1] * inv; az = s[2] * inv;
    gx = s[3] * inv; gy = s[4] * inv; gz = s[5] * inv;
  }

  uint32_t now_us = micros();
  float dt = (now_us - last_filter_us) * 1e-6f;
  last_filter_us = now_us;

  // 2. Encoder deltas. The baselines advance even when this tick is skipped, otherwise the next tick would see the
  //    ticks of several periods divided by one dt and produce a velocity spike (a position jump).
  int32_t enc_l = (S_FL * enc_fl.getCount() + S_RL * enc_rl.getCount()) / 2;
  int32_t enc_r = (S_FR * enc_fr.getCount() + S_RR * enc_rr.getCount()) / 2;
  float d_left  = (enc_l - last_enc_l) * M_PER_TICK;
  float d_right = (enc_r - last_enc_r) * M_PER_TICK;
  last_enc_l = enc_l;
  last_enc_r = enc_r;
  if (!(dt > 0.0f && dt < EST_DT_MAX)) return;

  // 3. Roll/pitch filter
  if (imu_ok) att.update(ax, ay, az, gx, gy, gz, dt);

  // 4. Zero velocity update + gyro bias learning. Bias is learned ONLY while commanded still (a skid-steer robot that
  //    veers slightly would otherwise teach real rotation into the bias while driving "straight").
  static uint32_t still_since = 0;
  bool still = fabsf(cmd_lin) < 0.003f && fabsf(cmd_ang) < 0.01f;
  if (still) { if (still_since == 0) still_since = now_ms; }
  else       { still_since = 0; }
  g_still_ms = still ? (now_ms - still_since) : 0;

  if (imu_ok && still && g_still_ms > 1000 && fabsf(gz) < 0.05f) {
    imuSensor.updateGyroBiasZ(imuSensor.getGyroBiasZ() + 0.02f * gz);
    gz -= 0.02f * gz;   // apply instantly to this tick
  }
  if (fabsf(cmd_ang) < 0.01f && fabsf(gz) < 0.005f) gz = 0.0f;   // deadband only while no turn is commanded

  // 5. IMU outage handling: hold the last rate for up to 250 ms, then fall back to encoder yaw (much worse than the gyro)
  static float gz_last = 0.0f;
  static uint8_t imu_miss = 0;
  if (imu_ok) { imu_miss = 0; gz_last = gz; }
  else {
    if (imu_miss < 255) imu_miss++;
    gz = (imu_miss <= 5) ? gz_last : (d_right - d_left) / TRACK_CMD / dt;
  }

  float d_center = (d_left + d_right) / 2.0f;
  float v_enc = d_center / dt;   // yaw from the encoders is not used while the gyro is healthy (skid steer under-reads turns)
  float accel_forward = imu_ok ? att.forwardAccel() : 0.0f;

  // 6. EKF prediction
  float v_pred = X(3) + (accel_forward * dt);
  float theta_pred = X(2) + (gz * dt);
  if (still) v_pred = 0.0f;   // ZUPT: kill accelerometer drift when commanded to stop

  X(0) += v_pred * cosf(theta_pred) * dt;
  X(1) += v_pred * sinf(theta_pred) * dt;
  X(2) = theta_pred;
  X(3) = v_pred;

  BLA::Matrix<4, 4> F = {
      1.0f, 0.0f, -v_pred * sinf(theta_pred) * dt, cosf(theta_pred) * dt,
      0.0f, 1.0f,  v_pred * cosf(theta_pred) * dt, sinf(theta_pred) * dt,
      0.0f, 0.0f,  1.0f,                           0.0f,
      0.0f, 0.0f,  0.0f,                           1.0f
  };
  P = F * P * ~F + Q;
  if (imu_miss > 5) P(2, 2) += 1e-4f;   // encoder yaw is much worse than the gyro: grow yaw covariance faster

  // 7. Measurement update: encoder forward velocity only, H = [0 0 0 1] (scalar update, no matrix inverse)
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

  // 8. Heading hold
  if (!imu_ok && hold_active) { hold_active = false; hold_corr = 0.0f; apply_drive(); }
  if (hold_active && (now_ms - last_cmd_ms <= CMD_TIMEOUT_MS)) {
    float err = heading_target - filtered_yaw;
    while (err >  PI) err -= 2.0f * PI;
    while (err < -PI) err += 2.0f * PI;
    hold_corr = constrain(HOLD_KP * err - HOLD_KD * gz, -HOLD_MAX, HOLD_MAX);
    apply_drive();
  }

  // 9. Publish. The pose is the integral up to the end of this window, so it is stamped "now" (no lag compensation).
  if (!(connected && time_synced)) return;

  int64_t t_ns = rmw_uros_epoch_nanos();
  int32_t t_sec = (int32_t)(t_ns / 1000000000LL);
  uint32_t t_nsec = (uint32_t)(t_ns % 1000000000LL);
  float cy = cosf(filtered_yaw * 0.5f), sy = sinf(filtered_yaw * 0.5f);

  static uint8_t odom_cnt = 0, imu_cnt = 0;

  if (++odom_cnt >= ODOM_PUB_DIV) {
    odom_cnt = 0;
    odom_msg.header.stamp.sec = t_sec;
    odom_msg.header.stamp.nanosec = t_nsec;
    odom_msg.pose.pose.position.x = X(0);
    odom_msg.pose.pose.position.y = X(1);
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sy;
    odom_msg.pose.pose.orientation.w = cy;
    odom_msg.twist.twist.linear.x = X(3);
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = gz;
    // Constant covariance entries are set once in setup(); only the varying ones are written here.
    odom_msg.pose.covariance[0]  = P(0, 0); odom_msg.pose.covariance[1]  = P(0, 1);
    odom_msg.pose.covariance[6]  = P(1, 0); odom_msg.pose.covariance[7]  = P(1, 1);
    odom_msg.pose.covariance[35] = P(2, 2);
    odom_msg.twist.covariance[0] = P(3, 3);
    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
  }

  if (imu_ok && ++imu_cnt >= IMU_PUB_DIV) {   // never publish stale IMU data
    imu_cnt = 0;
    imu_msg.header.stamp.sec = t_sec;
    imu_msg.header.stamp.nanosec = t_nsec;
    imu_msg.linear_acceleration.x = ax * att.gainK();
    imu_msg.linear_acceleration.y = ay * att.gainK();
    imu_msg.linear_acceleration.z = az * att.gainK();
    imu_msg.angular_velocity.x = gx - att.gyroBias(0);
    imu_msg.angular_velocity.y = gy - att.gyroBias(1);
    imu_msg.angular_velocity.z = gz;

    float cr = cosf(att.roll * 0.5f),  sr = sinf(att.roll * 0.5f);
    float cp = cosf(att.pitch * 0.5f), sp = sinf(att.pitch * 0.5f);
    imu_msg.orientation.w = cr*cp*cy + sr*sp*sy;
    imu_msg.orientation.x = sr*cp*cy - cr*sp*sy;
    imu_msg.orientation.y = cr*sp*cy + sr*cp*sy;
    imu_msg.orientation.z = cr*cp*sy - sr*sp*cy;
    RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
  }
}

void setup() {
  i2c_mtx = xSemaphoreCreateMutex();

  Serial.setRxBufferSize(SERIAL_RX_BUF);   // must be called before begin()
  Serial.begin(MICROROS_BAUD);
  set_microros_serial_transports(Serial);
  analogWriteFrequency(20000);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2);
  tilt_servo.setPeriodHertz(50); dump_servo.setPeriodHertz(50);
  tilt_servo.attach(TILT_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  dump_servo.attach(DUMP_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  tilt_servo.write(90); dump_servo.write(15);

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
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  if (tcs.begin()) color_active = true;

  if (imuSensor.begin()) {
    imu_active = true;
    imuSensor.calibrateGyro();   // robot must be still (~1.2 s)
    attitudeCalibrate();         // robot must be still and level (~2 s)
    last_filter_us = micros();
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, NULL, 2, NULL, 0);
  }

  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame);

  imu_msg.orientation_covariance[0] = 0.01;   // roll: from attitude filter
  imu_msg.orientation_covariance[4] = 0.01;   // pitch: from attitude filter
  imu_msg.orientation_covariance[8] = 1e3;    // yaw: gyro-integrated, not an absolute heading
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

  // Constant odometry covariance entries (row-major 6x6: x=0, y=7, z=14, roll=21, pitch=28, yaw=35). Unobserved axes are huge.
  for (int i = 0; i < 36; i++) { odom_msg.pose.covariance[i] = 0.0; odom_msg.twist.covariance[i] = 0.0; }
  odom_msg.pose.covariance[14] = odom_msg.pose.covariance[21] = odom_msg.pose.covariance[28] = 1e6;
  odom_msg.twist.covariance[7] = odom_msg.twist.covariance[14] = odom_msg.twist.covariance[21] = odom_msg.twist.covariance[28] = 1e6;
  odom_msg.twist.covariance[35] = 0.05;   // wz, PLACEHOLDER

  agent_state = WAITING_AGENT;
}

void loop() {
  uint32_t current_time = millis();

  static uint32_t last_agent_check = 0;
  static uint8_t ping_fail = 0;
  switch (agent_state) {
    case WAITING_AGENT:
      if (current_time - last_agent_check >= 500) {
        last_agent_check = current_time;
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
      if (current_time - last_agent_check >= PING_PERIOD_MS) {
        last_agent_check = current_time;
        if (rmw_uros_ping_agent(PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
          ping_fail = 0;
        } else if (++ping_fail >= PING_MAX_FAILS) {
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
  current_time = millis();

  if (run_selfcheck) {
    run_selfcheck = false;
    selfcheck_run();
    current_time = millis();
  }

  static uint32_t last_speed_ms = 0;
  if (current_time - last_speed_ms >= SPEED_LOOP_MS) {
    float dt = (current_time - last_speed_ms) * 1e-3f;
    last_speed_ms = current_time;
    if (dt > 0.25f) speed_ctrl_reset();   // first tick after a long block: just re-sync
    else            speed_ctrl_step(dt);
  }

  // Time sync: retry until it succeeds. Once running, re-sync only while the robot is standing still, because a sync
  // can step the epoch offset and block the loop for up to 200 ms.
  if (connected && !time_synced) {
    if (current_time - last_sync_ms >= TIME_RETRY_MS) {
      last_sync_ms = current_time;
      rmw_uros_sync_session(TIME_RETRY_TIMEOUT);
      time_synced = rmw_uros_epoch_synchronized();
    }
  } else if (connected && g_still_ms > 2000 && current_time - last_sync_ms >= TIME_RESYNC_MS) {
    last_sync_ms = current_time;
    rmw_uros_sync_session(200);
    time_synced = rmw_uros_epoch_synchronized();
  }

  static uint32_t last_sc_pub = 0;
  if (connected && sc_enabled && current_time - last_sc_pub >= 1000) {
    last_sc_pub = current_time;
    selfcheck_publish();
  }

  if (abs(current_conveyor_cmd) > 0.1) {
    conveyorStepper.enableOutputs(); conveyorStepper.runSpeed();
  } else {
    conveyorStepper.disableOutputs();
  }

  static uint32_t last_est_ms = 0;
  if (current_time - last_est_ms >= EST_PERIOD_MS) {
    last_est_ms = current_time;
    estimator_tick(current_time, connected);
  }

  static uint32_t last_enc_pub = 0;
  if (connected && time_synced && current_time - last_enc_pub >= ENC_PUB_MS) {
    last_enc_pub = current_time;
    enc_msg.data.data[0] = (int32_t)enc_fl.getCount();
    enc_msg.data.data[1] = (int32_t)enc_rl.getCount();
    enc_msg.data.data[2] = (int32_t)enc_fr.getCount();
    enc_msg.data.data[3] = (int32_t)enc_rr.getCount();
    int64_t enc_t_ns = rmw_uros_epoch_nanos();
    enc_msg.data.data[4] = (int32_t)(enc_t_ns / 1000000000LL);
    enc_msg.data.data[5] = (int32_t)(enc_t_ns % 1000000000LL);
    RCSOFTCHECK(rcl_publish(&enc_pub, &enc_msg, NULL));
  }

  static uint32_t last_color_pub = 0;
  static bool color_primed = false;
  if (connected && color_active && current_time - last_color_pub >= 100) {
    last_color_pub = current_time;
    uint16_t r, g, b;
    if (tcs_read(r, g, b)) {
      if (!color_primed) {
        color_primed = true;
      } else {
        color_msg.x = r; color_msg.y = g; color_msg.z = b;
        RCSOFTCHECK(rcl_publish(&color_pub, &color_msg, NULL));
      }
    }
  }

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