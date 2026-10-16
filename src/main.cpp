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
// Published by the ESP32 (best effort):
//   /encoders           std_msgs/Int32MultiArray  20 Hz  [FL, RL, FR, RR ticks, stamp_sec, stamp_nanosec]
//   /imu_data           sensor_msgs/Imu           20 Hz  frame imu_link; roll/pitch from attitude filter, yaw from EKF
//   /odometry/filtered  nav_msgs/Odometry         20 Hz  odom -> base_link; EKF pose + velocity (this is Nav2's odom)
//   /trash_color        geometry_msgs/Vector3     10 Hz  TCS34725 raw R, G, B
//   /self_check         std_msgs/Int32             1 Hz  2 bits per item (0 not run, 1 pass, 2 fail, 3 check by eye), bit 30 = run toggle
// Subscribed by the ESP32:
//   /cmd_vel_raw        geometry_msgs/Twist       drive command in m/s and rad/s (from twist_mux); angular.x > 0.5 asks for heading hold
//   /pan_tilt_cmd       geometry_msgs/Point       y = tilt servo angle in degrees (x is ignored, there is no pan servo)
//   /dump_cmd           std_msgs/Float32          dump servo angle, 15..165 degrees
//   /conveyor_cmd       std_msgs/Float32          conveyor stepper speed in steps/s (+-700)
//   /self_check_cmd     std_msgs/Int32            any message starts the self-check (ignored while driving)
//   /heading_hold_cmd   std_msgs/Bool             master switch for heading hold (true again after every agent reconnect)
//   /max_speed_cmd      std_msgs/Float32          no longer used, kept so old clients do not break
// =========================================================================================

// ---------- Robot Kinematics (Tune these in meters) ----------
const float WHEEL_RADIUS = 0.0425f; // From Python file: 0.065m diameter
const float TRACK_WIDTH = 0.215f;
const float TICKS_PER_REV = 748.0f; // Half-quad counts per wheel rev
const float M_PER_TICK = (2.0f * PI * WHEEL_RADIUS) / TICKS_PER_REV;

// ---------- Wheel speed PI ----------
#define SPEED_LOOP_MS 50            // must equal the sample time the gains were tuned at (SAMPLE_DT = 0.05 s)
const float MAX_WHEEL_RPM = 70.0f;  // wheel rpm at normalised speed 1.0 (tuning dashboard range was 0..70 rpm)

// ---------- cmd_vel is in real units (m/s, rad/s) so Nav2 can drive it directly ----------
const float WHEEL_CIRC      = 2.0f * PI * WHEEL_RADIUS;            // 0.2042 m
const float MAX_WHEEL_SPEED = MAX_WHEEL_RPM / 60.0f * WHEEL_CIRC;  // 0.238 m/s at 70 rpm
// Effective track for turn commands. Skid steer turns as if the track were wider than the geometric 0.20 m.
// Tune: command 1.0 rad/s, read /imu_data angular_velocity.z, TRACK_CMD_new = TRACK_CMD * 1.0 / measured.
const float TRACK_CMD = 0.30f;

// Wheel labels are from the robot's own perspective (standing behind it, looking the way it drives).
// Sign that makes each wheel's encoder count positive when the robot rolls forward.
// Encoder sign for odometry (counts positive when the wheel rolls forward). It must equal MOTOR_DIR_*: the encoders count
// up when a motor is driven with positive raw PWM (that is what PID_TEST.cpp relies on), and forward is raw sign MOTOR_DIR_*.
// Hard-coding it separately (it was -1,-1,+1,+1) disagreed with pins.h on the FRONT wheels.
const int S_FL = MOTOR_DIR_FL, S_RL = MOTOR_DIR_RL, S_FR = MOTOR_DIR_FR, S_RR = MOTOR_DIR_RR;

// ---------- EKF Matrices (4-State) ----------
BLA::Matrix<4> X = {0.0f, 0.0f, 0.0f, 0.0f}; // State vector [x, y, theta, v]
BLA::Matrix<4, 4> P = {1.0f, 0.0f, 0.0f, 0.0f,
                       0.0f, 1.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 1.0f, 0.0f,
                       0.0f, 0.0f, 0.0f, 1.0f}; // Covariance Matrix

BLA::Matrix<4, 4> Q = {0.001f, 0.0f,   0.0f,   0.0f,
                       0.0f,   0.001f, 0.0f,   0.0f,
                       0.0f,   0.0f,   0.005f, 0.0f,
                       0.0f,   0.0f,   0.0f,   0.05f}; // Process Noise (Gyro & Accel trust)

BLA::Matrix<4, 4> R = {0.05f, 0.0f,  0.0f,  0.0f,
                       0.0f,  0.05f, 0.0f,  0.0f,
                       0.0f,  0.0f,  0.1f,  0.0f,    // R(2,2): unused while Y(2) is forced to 0 below (yaw is gyro-only)
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

#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2400

const int TCS_ADDR = 0x29;

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
geometry_msgs__msg__Point pan_msg;
rcl_subscription_t dump_sub;
std_msgs__msg__Float32 dump_msg;
rcl_subscription_t conv_sub;
std_msgs__msg__Float32 conv_msg;
rcl_subscription_t speed_sub;
std_msgs__msg__Float32 speed_msg;

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
  cmd_lin = msg->linear.x;     // m/s
  cmd_ang = msg->angular.z;    // rad/s

  // Heading hold only when the sender asks for it (dashboard sets angular.x = 1, Nav2 leaves it 0)
  bool hold_req  = msg->angular.x > 0.5f;
  bool want_hold = hold_enabled && hold_req && imu_active && fabsf(cmd_ang) < 0.02f && fabsf(cmd_lin) > 0.02f;
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

void speed_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  (void)msg;   // kept so old clients publishing /max_speed_cmd do not break; speed limiting is on the sender side
}

void hold_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  hold_enabled = msg->data;
  if (!hold_enabled) { hold_active = false; hold_corr = 0.0f; }
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
  enc.setCount(c0);   // TEMP: encode both legs in the readout
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
  selfcheck_publish();
}

// ---------- micro-ROS connection management ----------
// Serial link budget: odometry+imu+encoders+color at 20/20/20/10 Hz need about 24 kB/s. 115200 baud carries only
// 11.5 kB/s, so the link saturates and a ping times out -> the session is dropped and re-created ("random restarts").
// MICROROS_BAUD must equal the agent's --baudrate (agent: serial --dev /dev/ttyUSBx -b 921600).
#define MICROROS_BAUD      460800
#define IMU_ODOM_PUB_DIV   1      // publish imu+odom every N-th 50 ms cycle (1 = 20 Hz). Use >= 4 if you must stay at 115200.
#define PING_PERIOD_MS     500    // agent liveness check interval
#define PING_TIMEOUT_MS    200    // per-ping timeout
#define PING_MAX_FAILS     4      // consecutive failed pings before the session is dropped (about 3 s)
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;
bool hold_sub_ok = false;
#define RC_OK(fn) { if ((fn) != RCL_RET_OK) return false; }

static bool create_entities() {
  allocator = rcl_get_default_allocator();
  RC_OK(rclc_support_init(&support, 0, NULL, &allocator));
  RC_OK(rclc_node_init_default(&node, "base_controller", "", &support));

  RC_OK(rclc_publisher_init_best_effort(&enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "encoders"));
  RC_OK(rclc_publisher_init_best_effort(&color_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "trash_color"));
  RC_OK(rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_data"));
  RC_OK(rclc_publisher_init_best_effort(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odometry/filtered"));

  rmw_uros_sync_session(1000);

  RC_OK(rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel_raw"));
  RC_OK(rclc_subscription_init_default(&pan_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "pan_tilt_cmd"));
  RC_OK(rclc_subscription_init_default(&dump_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "dump_cmd"));
  RC_OK(rclc_subscription_init_default(&conv_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "conveyor_cmd"));
  RC_OK(rclc_subscription_init_default(&speed_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "max_speed_cmd"));

  RC_OK(rclc_executor_init(&executor, &support.context, 7, &allocator));
  RC_OK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &pan_sub, &pan_msg, &pan_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &dump_sub, &dump_msg, &dump_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &conv_sub, &conv_msg, &conveyor_callback, ON_NEW_DATA));
  RC_OK(rclc_executor_add_subscription(&executor, &speed_sub, &speed_msg, &speed_callback, ON_NEW_DATA));

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
  (void)rcl_subscription_fini(&speed_sub, &node);
  (void)rcl_subscription_fini(&sc_sub, &node);
  (void)rcl_subscription_fini(&hold_sub, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);

  sc_enabled = false;
  hold_sub_ok = false;
  hold_enabled = true;
}

void setup() {
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
  if (tcs.begin()) color_active = true;

  if (imuSensor.begin()) {
    imu_active = true;
    imuSensor.calibrateGyro();   // robot must be still (~1.2 s)
    attitudeCalibrate();         // robot must be still and level (~2 s)
    last_filter_us = micros();
  }

  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame);

  imu_msg.orientation_covariance[0] = 0.01;   // roll: from attitude filter
  imu_msg.orientation_covariance[4] = 0.01;   // pitch: from attitude filter
  imu_msg.orientation_covariance[8] = 0.1;
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
    if (dt > 0.25f) speed_ctrl_reset();   // first tick after a long block (self-check): just re-sync
    else            speed_ctrl_step(dt);
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

  static uint32_t last_enc_pub = 0;
  if (connected && current_time - last_enc_pub >= 50) {
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

  static uint32_t last_imu_pub = 0;
  if (current_time - last_imu_pub >= 50) {
    last_imu_pub = current_time;

    float ax, ay, az, gx, gy, gz;
    if (imu_active && imuSensor.readData(ax, ay, az, gx, gy, gz)) {
      // Sensor frame is standard FLU (x fwd, y left, z up): no axis flips.

      uint32_t now_us = micros();
      float dt = (now_us - last_filter_us) * 1e-6f;
      last_filter_us = now_us;

      if (dt > 0.0f && dt < 0.5f) {

        // Roll/pitch filter + ramp flag. Runs on the driver-corrected gz,
        // before ZUPT/deadband modify it.
        att.update(ax, ay, az, gx, gy, gz, dt);

        // --- Zero Velocity Update ---
        static uint32_t still_since = 0;
        bool still = fabsf(cmd_lin) < 0.003f && fabsf(cmd_ang) < 0.01f;   // m/s, rad/s (lowest moving command is about 0.0017 m/s)
        if (still) {
          if (still_since == 0) still_since = current_time;
          if (current_time - still_since > 1000 && fabsf(gz) < 0.05f) {
            float current_bias = imuSensor.getGyroBiasZ();
            imuSensor.updateGyroBiasZ(current_bias + (0.02f * gz));
            gz -= (0.02f * gz); // Apply instantly to this tick
          }
        } else {
          still_since = 0;
        }

        if (fabsf(gz) < 0.005f) gz = 0.0f;

        // --- EKF Update Process (4-State) ---
        int32_t enc_l = (S_FL * enc_fl.getCount() + S_RL * enc_rl.getCount()) / 2;
        int32_t enc_r = (S_FR * enc_fr.getCount() + S_RR * enc_rr.getCount()) / 2;

        float d_left = (enc_l - last_enc_l) * M_PER_TICK;
        float d_right = (enc_r - last_enc_r) * M_PER_TICK;
        last_enc_l = enc_l;
        last_enc_r = enc_r;

        float d_center = (d_left + d_right) / 2.0f;
        float v_enc = d_center / dt;
        float d_theta_enc = (d_right - d_left) / TRACK_WIDTH;

        // Gravity-compensated forward accel in m/s^2 (gain-corrected, pitch-compensated)
        float accel_forward = att.forwardAccel();

        // 1. Prediction Step
        float v_pred = X(3) + (accel_forward * dt);
        float theta_pred = X(2) + (gz * dt);

        // ZUPT: Kill accelerometer drift when commanded to stop
        if (still) {
            v_pred = 0.0f;
        }

        X(0) += v_pred * cosf(theta_pred) * dt;
        X(1) += v_pred * sinf(theta_pred) * dt;
        X(2) = theta_pred;
        X(3) = v_pred;

        // 2. Jacobian Matrix F
        BLA::Matrix<4, 4> F = {
            1.0f, 0.0f, -v_pred * sinf(theta_pred) * dt, cosf(theta_pred) * dt,
            0.0f, 1.0f,  v_pred * cosf(theta_pred) * dt, sinf(theta_pred) * dt,
            0.0f, 0.0f,  1.0f,                           0.0f,
            0.0f, 0.0f,  0.0f,                           1.0f
        };

        P = F * P * ~F + Q;

        // 3. Measurement Update
        BLA::Matrix<4> Z = { X(0), X(1), X(2) - (gz * dt) + d_theta_enc, v_enc };
        BLA::Matrix<4, 4> H = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        };

        BLA::Matrix<4> Y = Z - (H * X);
        while (Y(2) > PI)  Y(2) -= 2.0f * PI;
        while (Y(2) < -PI) Y(2) += 2.0f * PI;
        Y(2) = 0.0f;   // yaw from the gyro only: skid-steer encoders under-read turns and would bias the heading

        // Consistency gate: skid-steer encoders can under-read yaw but cannot honestly over-read it.
        // If the encoder heading change exceeds what the gyro allows, the wheels are slipping or off the
        // ground (e.g. on a test rig), so ignore the encoder heading for this tick.
        BLA::Matrix<4, 4> Rk = R;
        if (fabsf(d_theta_enc) > 1.2f * fabsf(gz * dt) + 0.002f) Rk(2, 2) = 1e3f;
        BLA::Matrix<4, 4> S = H * P * ~H + Rk;
        BLA::Matrix<4, 4> K = P * ~H * Inverse(S);

        X += K * Y;
        while (X(2) > PI)  X(2) -= 2.0f * PI;
        while (X(2) < -PI) X(2) += 2.0f * PI;

        BLA::Matrix<4, 4> I = {1.0f,0.0f,0.0f,0.0f,
                               0.0f,1.0f,0.0f,0.0f,
                               0.0f,0.0f,1.0f,0.0f,
                               0.0f,0.0f,0.0f,1.0f};
        P = (I - K * H) * P;

        filtered_yaw = X(2);

        // --- Heading Hold Control ---
        if (hold_active && (current_time - last_cmd_ms <= CMD_TIMEOUT_MS)) {
          float err = heading_target - filtered_yaw;
          while (err >  PI) err -= 2.0f * PI;
          while (err < -PI) err += 2.0f * PI;
          hold_corr = constrain(HOLD_KP * err - HOLD_KD * gz, -HOLD_MAX, HOLD_MAX);
          apply_drive();
        }

        // --- Publish IMU ---
        int64_t t_ns = rmw_uros_epoch_nanos();
        imu_msg.header.stamp.sec = (int32_t)(t_ns / 1000000000LL);
        imu_msg.header.stamp.nanosec = (uint32_t)(t_ns % 1000000000LL);

        imu_msg.linear_acceleration.x = ax * att.gainK();
        imu_msg.linear_acceleration.y = ay * att.gainK();
        imu_msg.linear_acceleration.z = az * att.gainK();
        imu_msg.angular_velocity.x = gx - att.gyroBias(0);
        imu_msg.angular_velocity.y = gy - att.gyroBias(1);
        imu_msg.angular_velocity.z = gz;

        // Orientation: roll/pitch from attitude filter, yaw from EKF (ZYX Euler -> quaternion)
        float cr = cosf(att.roll * 0.5f),  sr = sinf(att.roll * 0.5f);
        float cp = cosf(att.pitch * 0.5f), sp = sinf(att.pitch * 0.5f);
        float cy = cosf(filtered_yaw * 0.5f), sy = sinf(filtered_yaw * 0.5f);
        imu_msg.orientation.w = cr*cp*cy + sr*sp*sy;
        imu_msg.orientation.x = sr*cp*cy - cr*sp*sy;
        imu_msg.orientation.y = cr*sp*cy + sr*cp*sy;
        imu_msg.orientation.z = cr*cp*sy - sr*sp*cy;

        // --- Publish Odometry ---
        odom_msg.header.stamp.sec = imu_msg.header.stamp.sec;
        odom_msg.header.stamp.nanosec = imu_msg.header.stamp.nanosec;

        odom_msg.pose.pose.position.x = X(0);
        odom_msg.pose.pose.position.y = X(1);
        odom_msg.pose.pose.position.z = 0.0;

        // Yaw-only orientation for the 2D odometry
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sy;
        odom_msg.pose.pose.orientation.w = cy;

        // Broadcast the fused EKF velocity instead of raw encoder velocity
        odom_msg.twist.twist.linear.x = X(3);
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = gz;

        // Pose covariance (row-major 6x6: x=0, y=7, yaw=35). Unobserved axes get a large value.
        for (int i = 0; i < 36; i++) { odom_msg.pose.covariance[i] = 0.0; odom_msg.twist.covariance[i] = 0.0; }
        odom_msg.pose.covariance[0]  = P(0, 0); odom_msg.pose.covariance[1]  = P(0, 1);
        odom_msg.pose.covariance[6]  = P(1, 0); odom_msg.pose.covariance[7]  = P(1, 1);
        odom_msg.pose.covariance[35] = P(2, 2);
        odom_msg.pose.covariance[14] = odom_msg.pose.covariance[21] = odom_msg.pose.covariance[28] = 1e6;
        odom_msg.twist.covariance[0]  = P(3, 3); // Variance of linear velocity
        odom_msg.twist.covariance[35] = 0.05;    // wz, PLACEHOLDER
        odom_msg.twist.covariance[7]  = odom_msg.twist.covariance[14] = odom_msg.twist.covariance[21] = odom_msg.twist.covariance[28] = 1e6;

        static uint8_t pub_cnt = 0;
        if (connected && (++pub_cnt >= IMU_ODOM_PUB_DIV)) {
            pub_cnt = 0;
            RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
            RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
        }
      }
    }
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

  static uint32_t last_sync_ms = 0;
  if (connected && current_time - last_sync_ms >= 300000UL) {
    last_sync_ms = current_time;
    rmw_uros_sync_session(200);
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