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
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>

#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

#include "pins.h"

// ---------- Hardware Instances ----------
Servo pan_servo, tilt_servo, dump_servo;
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;
AccelStepper conveyorStepper(AccelStepper::FULL4WIRE, CONV_IN1, CONV_IN3, CONV_IN2, CONV_IN4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2400

const int MPU_ADDR = 0x68;
const int TCS_ADDR = 0x29;

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

rcl_publisher_t enc_pub;
std_msgs__msg__Int32MultiArray enc_msg;
int32_t enc_data[4];

rcl_publisher_t color_pub;
geometry_msgs__msg__Vector3 color_msg;

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

// ---------- State Variables ----------
volatile uint32_t last_cmd_ms = 0;
float max_linear_vel = 0.5;
float current_conveyor_cmd = 0.0;
uint32_t last_dump_cmd_ms = 0;
bool dump_servo_attached = true;

bool color_active = false;
bool imu_active = false;

// ---------- Yaw estimate ----------
float gyro_bias_z = 0.0f;   // rad/s, measured at boot
float filtered_yaw = 0.0f;  // rad, bias-corrected gyro integral (still drifts slowly)
uint32_t last_filter_us = 0;
static char imu_frame[] = "imu_link";

// ---------- Heading hold ----------
#define HOLD_KP   1.5f     // normalised wheel cmd per rad of heading error; tune
#define HOLD_KD   0.15f    // damping on gyro z; tune
#define HOLD_MAX  0.3f     // max correction, normalised
float cmd_lin = 0.0f, cmd_ang = 0.0f;
float heading_target = 0.0f;
float hold_corr = 0.0f;
bool hold_active = false;

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

void drive_motors(float left_speed, float right_speed) {
  float left_norm  = constrain(left_speed,  -1.0f, 1.0f) * max_linear_vel;
  float right_norm = constrain(right_speed, -1.0f, 1.0f) * max_linear_vel;
  set_motor(FL_IN1, FL_IN2, left_norm  * MOTOR_DIR_FL);
  set_motor(RL_IN1, RL_IN2, left_norm  * MOTOR_DIR_RL);
  set_motor(FR_IN1, FR_IN2, right_norm * MOTOR_DIR_FR);
  set_motor(RR_IN1, RR_IN2, right_norm * MOTOR_DIR_RR);
}

// ---------- ROS 2 Callbacks ----------
static void apply_drive() {
  float w = cmd_ang + hold_corr;
  drive_motors(cmd_lin - w, cmd_lin + w);
}

void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_ms = millis();
  cmd_lin = msg->linear.x;
  cmd_ang = msg->angular.z;

  // Hold heading only while driving straight and the gyro is available
  bool want_hold = imu_active && fabsf(cmd_ang) < 0.02f && fabsf(cmd_lin) > 0.02f;
  if (want_hold && !hold_active) heading_target = filtered_yaw;   // latch on entry
  if (!want_hold) hold_corr = 0.0f;
  hold_active = want_hold;

  apply_drive();
}

void pan_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  pan_servo.write((int)constrain(msg->x, PAN_MIN, PAN_MAX));
  tilt_servo.write((int)constrain(msg->y, TILT_MIN, TILT_MAX));
}

void dump_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  if (!dump_servo_attached) {
    dump_servo.attach(DUMP_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    dump_servo_attached = true;
  }
  dump_servo.write((int)constrain(msg->data, 5, 175));
  last_dump_cmd_ms = millis();
}

void conveyor_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  current_conveyor_cmd = msg->data;
  conveyorStepper.setSpeed(current_conveyor_cmd);
}

void speed_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  max_linear_vel = constrain(msg->data, 0.1f, 1.0f);
}

// ---------- I2C helpers (single-threaded, no mutex needed) ----------
static bool mpu_read(int16_t v[7]) {   // ax ay az temp gx gy gz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();          // explicit sequencing: high byte first
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

static void mpu_setup_filter() {
  Wire.beginTransmission(MPU_ADDR);    // CONFIG reg: DLPF ~44 Hz, cuts motor vibration
  Wire.write(0x1A); Wire.write(0x03);
  Wire.endTransmission(true);

  double sum = 0; int n = 0;           // ~2 s, robot must be still
  for (int i = 0; i < 400; i++) {
    int16_t v[7];
    if (mpu_read(v)) { sum += v[6]; n++; }
    delay(3);
  }
  if (n < 100) { imu_active = false; return; }
  gyro_bias_z = (float)(sum / n) / 131.0f * (PI / 180.0f);
  last_filter_us = micros();
}

// Reads the latest integration result. Does not wait: the sensor integrates continuously.
static bool tcs_read(uint16_t &r, uint16_t &g, uint16_t &b) {
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(0xB4);                    // CMD bit | auto-increment | CDATAL (0x14)
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(TCS_ADDR, 8, true) != 8) return false;
  uint8_t d[8];
  for (int i = 0; i < 8; i++) d[i] = Wire.read();
  // order: clear, red, green, blue; each little-endian
  r = (uint16_t)(d[2] | (d[3] << 8));
  g = (uint16_t)(d[4] | (d[5] << 8));
  b = (uint16_t)(d[6] | (d[7] << 8));
  return true;
}

// ---------- Self-check (manual only: runs solely on /self_check_cmd) ----------
// Result is published on /self_check as one Int32: 2 bits per item (item i at bits 2i..2i+1),
// bit 30 toggles each time a full run completes. Trigger: any Int32 on /self_check_cmd.
enum { SC_NOT_RUN = 0, SC_PASS = 1, SC_FAIL = 2, SC_UNVERIFIED = 3 };
enum { SC_MPU, SC_TCS, SC_MOT_FL, SC_MOT_RL, SC_MOT_FR, SC_MOT_RR,
       SC_PAN, SC_TILT, SC_DUMP, SC_STEPPER, SC_COUNT };
#define SC_MIN_COUNTS  5      // encoder counts a spinning wheel must produce; tune to your PPR
#define SC_TEST_SPEED  0.3f   // normalised, same scale as set_motor()

uint8_t sc_state[SC_COUNT] = {0};
bool sc_toggle = false;
bool sc_enabled = false;      // false if the extra pub/sub could not be created; base robot still runs
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
  if (hold_active || fabsf(cmd_lin) > 0.01f || fabsf(cmd_ang) > 0.01f) return;  // never start while driving
  run_selfcheck = true;
}

// WHO_AM_I + gravity magnitude + boot gyro offset. Robot must be still.
static uint8_t check_mpu() {
  if (!imu_active) return SC_FAIL;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return SC_FAIL;
  if (Wire.requestFrom(MPU_ADDR, 1, true) != 1) return SC_FAIL;
  uint8_t id = Wire.read();
  if (id != 0x68 && id != 0x70 && id != 0x71 && id != 0x72) return SC_FAIL;   // clones vary; check yours

  double amag = 0; int n = 0;
  for (int i = 0; i < 20; i++) {
    int16_t v[7];
    if (mpu_read(v)) {
      amag += sqrt((double)v[0] * v[0] + (double)v[1] * v[1] + (double)v[2] * v[2]);
      n++;
    }
    delay(5);
  }
  if (n < 15) return SC_FAIL;
  amag /= n;
  if (amag < 13100 || amag > 19660) return SC_FAIL;   // 0.8 to 1.2 g at the +/-2 g range
  if (fabsf(gyro_bias_z) > 0.35f) return SC_FAIL;     // more than ~20 deg/s zero-rate offset
  return SC_PASS;
}

// Chip ID register (0x12) only; says nothing about the optics or LED.
static uint8_t check_tcs() {
  if (!color_active) return SC_FAIL;
  Wire.beginTransmission(TCS_ADDR);
  Wire.write(0x92);                    // CMD bit | ID register 0x12
  if (Wire.endTransmission(false) != 0) return SC_FAIL;
  if (Wire.requestFrom(TCS_ADDR, 1, true) != 1) return SC_FAIL;
  uint8_t id = Wire.read();
  return (id == 0x44 || id == 0x4D || id == 0x10) ? SC_PASS : SC_FAIL;
}

// Spin forward then reverse; pass if the encoder moved both ways, in opposite directions.
// Cannot tell a dead motor from a dead encoder.
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
  bool ok = af >= SC_MIN_COUNTS && ar >= SC_MIN_COUNTS && ((f > 0) != (r > 0));
  return ok ? SC_PASS : SC_FAIL;
}

// Hobby servos and the open-loop stepper give no feedback: we can only move them.
static void exercise_actuators() {
  pan_servo.write((int)constrain(75, PAN_MIN, PAN_MAX));  delay(300);
  pan_servo.write((int)constrain(105, PAN_MIN, PAN_MAX)); delay(300);
  pan_servo.write(90);                                    delay(300);
  sc_state[SC_PAN] = SC_UNVERIFIED;

  tilt_servo.write((int)constrain(75, TILT_MIN, TILT_MAX));  delay(300);
  tilt_servo.write((int)constrain(105, TILT_MIN, TILT_MAX)); delay(300);
  tilt_servo.write(90);                                      delay(300);
  sc_state[SC_TILT] = SC_UNVERIFIED;

  if (!dump_servo_attached) {
    dump_servo.attach(DUMP_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    dump_servo_attached = true;
  }
  dump_servo.write(25); delay(500);     // small lift only, does not dump
  dump_servo.write(5);  delay(500);
  last_dump_cmd_ms = millis();
  sc_state[SC_DUMP] = SC_UNVERIFIED;

  conveyorStepper.enableOutputs();
  conveyorStepper.setSpeed(300);
  uint32_t t0 = millis();
  while (millis() - t0 < 600) conveyorStepper.runSpeed();
  conveyorStepper.setSpeed(current_conveyor_cmd);
  conveyorStepper.disableOutputs();
  sc_state[SC_STEPPER] = SC_UNVERIFIED;
}

// Blocks for roughly 7 s. Wheels must be off the ground.
static void selfcheck_run() {
  cmd_lin = 0.0f; cmd_ang = 0.0f;
  hold_active = false; hold_corr = 0.0f;
  drive_motors(0, 0);

  sc_state[SC_MPU] = check_mpu();
  sc_state[SC_TCS] = check_tcs();
  sc_state[SC_MOT_FL] = test_motor(FL_IN1, FL_IN2, enc_fl);
  sc_state[SC_MOT_RL] = test_motor(RL_IN1, RL_IN2, enc_rl);
  sc_state[SC_MOT_FR] = test_motor(FR_IN1, FR_IN2, enc_fr);
  sc_state[SC_MOT_RR] = test_motor(RR_IN1, RR_IN2, enc_rr);
  exercise_actuators();

  last_filter_us = micros();
  sc_toggle = !sc_toggle;
  selfcheck_publish();
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  analogWriteFrequency(20000);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1); ESP32PWM::allocateTimer(2);
  pan_servo.setPeriodHertz(50); tilt_servo.setPeriodHertz(50); dump_servo.setPeriodHertz(50);
  pan_servo.attach(PAN_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  tilt_servo.attach(TILT_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  dump_servo.attach(DUMP_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  pan_servo.write(90); tilt_servo.write(90); dump_servo.write(5);
  last_dump_cmd_ms = millis();

  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  drive_motors(0, 0);
  conveyorStepper.setMaxSpeed(700.0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B); enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B); enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);
  enc_msg.data.capacity = 4; enc_msg.data.size = 4; enc_msg.data.data = enc_data;

  Wire.begin(I2C_SDA, I2C_SCL);
  if (tcs.begin()) color_active = true;

  Wire.beginTransmission(MPU_ADDR);    // wake MPU6050
  Wire.write(0x6B); Wire.write(0);
  if (Wire.endTransmission(true) == 0) imu_active = true;
  if (imu_active) mpu_setup_filter();

  // Static IMU message fields (msg is global, so unset fields are already zero)
  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame);

  // Covariances are guesses; tune from logged stationary data
  imu_msg.orientation_covariance[0] = 1e3;   // roll not estimated
  imu_msg.orientation_covariance[4] = 1e3;   // pitch not estimated
  imu_msg.orientation_covariance[8] = 0.1;
  imu_msg.angular_velocity_covariance[0] = 0.001;
  imu_msg.angular_velocity_covariance[4] = 0.001;
  imu_msg.angular_velocity_covariance[8] = 0.001;
  imu_msg.linear_acceleration_covariance[0] = 0.05;
  imu_msg.linear_acceleration_covariance[4] = 0.05;
  imu_msg.linear_acceleration_covariance[8] = 0.05;

  delay(2000);
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { delay(100); }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_controller", "", &support));

  // Sensor publishers are best-effort so a full stream never blocks loop()
  RCCHECK(rclc_publisher_init_best_effort(&enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "encoders"));
  RCCHECK(rclc_publisher_init_best_effort(&color_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "trash_color"));
  RCCHECK(rclc_publisher_init_best_effort(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_data"));

  rmw_uros_sync_session(1000);         // result ignored; stamps fall back to local time

  RCCHECK(rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&pan_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "pan_tilt_cmd"));
  RCCHECK(rclc_subscription_init_default(&dump_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "dump_cmd"));
  RCCHECK(rclc_subscription_init_default(&conv_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "conveyor_cmd"));
  RCCHECK(rclc_subscription_init_default(&speed_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "max_speed_cmd"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pan_sub, &pan_msg, &pan_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &dump_sub, &dump_msg, &dump_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &conv_sub, &conv_msg, &conveyor_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &speed_sub, &speed_msg, &speed_callback, ON_NEW_DATA));

  // Self-check entities are created last and are non-fatal: if the micro-ROS entity limits
  // (RMW_UXRCE_MAX_SUBSCRIPTIONS defaults to 5) are hit, the base controller still runs.
  sc_enabled =
    rclc_publisher_init_best_effort(&sc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "self_check") == RCL_RET_OK &&
    rclc_subscription_init_default(&sc_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "self_check_cmd") == RCL_RET_OK &&
    rclc_executor_add_subscription(&executor, &sc_sub, &sc_cmd_msg, &selfcheck_callback, ON_NEW_DATA) == RCL_RET_OK;
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
  uint32_t current_time = millis();

  if (run_selfcheck) {               // manual trigger only
    run_selfcheck = false;
    selfcheck_run();
    current_time = millis();
  }
  static uint32_t last_sc_pub = 0;
  if (sc_enabled && current_time - last_sc_pub >= 1000) {   // republish so a late dashboard sees the last result
    last_sc_pub = current_time;
    selfcheck_publish();
  }

  if (dump_servo_attached && (current_time - last_dump_cmd_ms > 1000)) {
    dump_servo.detach();
    pinMode(DUMP_PIN, OUTPUT); digitalWrite(DUMP_PIN, LOW);
    dump_servo_attached = false;
  }

  if (abs(current_conveyor_cmd) > 0.1) {
    conveyorStepper.enableOutputs(); conveyorStepper.runSpeed();
  } else {
    conveyorStepper.disableOutputs();
  }

  // Encoders (20 Hz)
  static uint32_t last_enc_pub = 0;
  if (current_time - last_enc_pub >= 50) {
    last_enc_pub = current_time;
    enc_msg.data.data[0] = (int32_t)enc_fl.getCount();
    enc_msg.data.data[1] = (int32_t)enc_rl.getCount();
    enc_msg.data.data[2] = (int32_t)enc_fr.getCount();
    enc_msg.data.data[3] = (int32_t)enc_rr.getCount();
    RCSOFTCHECK(rcl_publish(&enc_pub, &enc_msg, NULL));
  }

  // IMU (20 Hz; ~300 B/msg, so don't raise the rate without raising the baud)
  static uint32_t last_imu_pub = 0;
  if (current_time - last_imu_pub >= 50) {
    last_imu_pub = current_time;
    int16_t v[7];
    if (imu_active && mpu_read(v)) {
      uint32_t now_us = micros();
      float dt = (now_us - last_filter_us) * 1e-6f;
      last_filter_us = now_us;

      if (dt > 0.0f && dt < 0.5f) {
        const float G = 9.80665f;
        float gz = (v[6] / 131.0f) * (PI / 180.0f) - gyro_bias_z;

        // Zero-velocity update: when commanded still for >1 s, slowly re-learn the bias
        static uint32_t still_since = 0;
        bool still = fabsf(cmd_lin) < 0.01f && fabsf(cmd_ang) < 0.01f;
        if (still) {
          if (still_since == 0) still_since = current_time;
          if (current_time - still_since > 1000 && fabsf(gz) < 0.05f) {
            gyro_bias_z += 0.02f * gz;
            gz = (v[6] / 131.0f) * (PI / 180.0f) - gyro_bias_z;
          }
        } else {
          still_since = 0;
        }

        if (fabsf(gz) < 0.005f) gz = 0.0f;   // ~0.3 deg/s residual-noise gate; tune

        filtered_yaw += gz * dt;
        if (filtered_yaw >  PI) filtered_yaw -= 2.0f * PI;
        if (filtered_yaw < -PI) filtered_yaw += 2.0f * PI;

        // Heading-hold PD: positive yaw = CCW, matching twist angular.z
        if (hold_active && (current_time - last_cmd_ms <= CMD_TIMEOUT_MS)) {
          float err = heading_target - filtered_yaw;
          while (err >  PI) err -= 2.0f * PI;
          while (err < -PI) err += 2.0f * PI;
          hold_corr = constrain(HOLD_KP * err - HOLD_KD * gz, -HOLD_MAX, HOLD_MAX);
          apply_drive();
        }

        int64_t t_ns = rmw_uros_epoch_nanos();
        imu_msg.header.stamp.sec = (int32_t)(t_ns / 1000000000LL);
        imu_msg.header.stamp.nanosec = (uint32_t)(t_ns % 1000000000LL);

        imu_msg.linear_acceleration.x = v[0] / 16384.0f * G;
        imu_msg.linear_acceleration.y = v[1] / 16384.0f * G;
        imu_msg.linear_acceleration.z = v[2] / 16384.0f * G;
        imu_msg.angular_velocity.x = (v[4] / 131.0f) * (PI / 180.0f);
        imu_msg.angular_velocity.y = (v[5] / 131.0f) * (PI / 180.0f);
        imu_msg.angular_velocity.z = gz;

        imu_msg.orientation.w = cosf(filtered_yaw * 0.5f);
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = sinf(filtered_yaw * 0.5f);

        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
      }
    }
  }

  // Colour (10 Hz): one 8-byte register burst, no waiting
  static uint32_t last_color_pub = 0;
  static bool color_primed = false;
  if (color_active && current_time - last_color_pub >= 100) {
    last_color_pub = current_time;
    uint16_t r, g, b;
    if (tcs_read(r, g, b)) {
      if (!color_primed) {
        color_primed = true;             // first sample after begin() can be zeros; drop it
      } else {
        color_msg.x = r; color_msg.y = g; color_msg.z = b;
        RCSOFTCHECK(rcl_publish(&color_pub, &color_msg, NULL));
      }
    }
  }

  // Re-sync agent clock every 5 min (blocks up to the timeout)
  static uint32_t last_sync_ms = 0;
  if (current_time - last_sync_ms >= 300000UL) {
    last_sync_ms = current_time;
    rmw_uros_sync_session(200);
  }

  // Command timeout failsafe
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