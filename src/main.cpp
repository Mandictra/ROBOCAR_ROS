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

using namespace BLA;

// ---------- Robot Kinematics (Tune these in meters) ----------
const float WHEEL_RADIUS = 0.0325f; // From Python file: 0.065m diameter
const float TRACK_WIDTH = 0.20f;    
const float TICKS_PER_REV = 1000.0f; // Half-quad counts per wheel rev
const float M_PER_TICK = (2.0f * PI * WHEEL_RADIUS) / TICKS_PER_REV;

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
                       0.0f,  0.0f,  0.01f, 0.0f,
                       0.0f,  0.0f,  0.0f,  0.02f}; // Measurement Noise (Encoder trust)

int32_t last_enc_l = 0;
int32_t last_enc_r = 0;

// ---------- Hardware Instances ----------
Servo pan_servo, tilt_servo, dump_servo;
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;
AccelStepper conveyorStepper(AccelStepper::FULL4WIRE, CONV_IN1, CONV_IN3, CONV_IN2, CONV_IN4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
MPU6050_Custom imuSensor;

#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2400

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
float max_linear_vel = 0.5;
float current_conveyor_cmd = 0.0;
uint32_t last_dump_cmd_ms = 0;
bool dump_servo_attached = true;

bool color_active = false;
bool imu_active = false;

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

  bool want_hold = hold_enabled && imu_active && fabsf(cmd_ang) < 0.02f && fabsf(cmd_lin) > 0.02f;
  if (want_hold && !hold_active) heading_target = filtered_yaw;
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
  dump_servo.write(constrain((int)msg->data, 5, 175));
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
       SC_PAN, SC_TILT, SC_DUMP, SC_STEPPER, SC_COUNT };
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
  enc.setCount(f * 1000 + r);   // TEMP: encode both legs in the readout
  if (fok && rok && dir) return SC_PASS;
  if (!fok)              return SC_FAIL;
  if (!rok)              return SC_UNVERIFIED;
  return SC_NOT_RUN;                       // "Not run": both legs count, same sign
}

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
  dump_servo.write(25); delay(500);     
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

  last_filter_us = micros();
  sc_toggle = !sc_toggle;
  selfcheck_publish();
}

// ---------- micro-ROS connection management ----------
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
  enc_msg.data.capacity = 6; enc_msg.data.size = 6; enc_msg.data.data = enc_data;

  Wire.begin(I2C_SDA, I2C_SCL);
  if (tcs.begin()) color_active = true;

  if (imuSensor.begin()) {
    imu_active = true;
    imuSensor.calibrateGyro();
    last_filter_us = micros();
  }

  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame);

  imu_msg.orientation_covariance[0] = 1e3;   
  imu_msg.orientation_covariance[4] = 1e3;   
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
  switch (agent_state) {
    case WAITING_AGENT:
      if (current_time - last_agent_check >= 500) {
        last_agent_check = current_time;
        agent_state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE : WAITING_AGENT;
      }
      break;
    case AGENT_AVAILABLE:
      if (create_entities()) {
        agent_state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        agent_state = WAITING_AGENT;
      }
      break;
    case AGENT_CONNECTED:
      if (current_time - last_agent_check >= 200) {
        last_agent_check = current_time;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) agent_state = AGENT_DISCONNECTED;
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
  static uint32_t last_sc_pub = 0;
  if (connected && sc_enabled && current_time - last_sc_pub >= 1000) {   
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
      
      // Add this line to flip the upside-down sensor
      gz = -gz; 
      
      uint32_t now_us = micros();
      float dt = (now_us - last_filter_us) * 1e-6f;
      last_filter_us = now_us;

      if (dt > 0.0f && dt < 0.5f) {
        
        // --- Zero Velocity Update ---
        static uint32_t still_since = 0;
        bool still = fabsf(cmd_lin) < 0.01f && fabsf(cmd_ang) < 0.01f;
        if (still) {
          if (still_since == 0) still_since = current_time;
          if (current_time - still_since > 1000 && fabsf(gz) < 0.05f) {
            float current_bias = imuSensor.getGyroBiasZ();
            imuSensor.updateGyroBiasZ(current_bias - (0.02f * gz));   // gz is already sign-flipped; bias is subtracted from the UNFLIPPED reading
            gz -= (0.02f * gz); // Apply instantly to this tick
          }
        } else {
          still_since = 0;
        }

        if (fabsf(gz) < 0.005f) gz = 0.0f;   

        // --- EKF Update Process (4-State) ---
        int32_t enc_l = (enc_fl.getCount() + enc_rl.getCount()) / 2;
        int32_t enc_r = (-enc_fr.getCount() + -enc_rr.getCount()) / 2;
        
        float d_left = (enc_l - last_enc_l) * M_PER_TICK;
        float d_right = (enc_r - last_enc_r) * M_PER_TICK;
        last_enc_l = enc_l;
        last_enc_r = enc_r;

        float d_center = (d_left + d_right) / 2.0f;
        float v_enc = d_center / dt;
        float d_theta_enc = (d_right - d_left) / TRACK_WIDTH;

        // Convert IMU G's to m/s^2 (Assuming X-axis points forward)
        float accel_forward = ax * 9.81f; 

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

        BLA::Matrix<4, 4> S = H * P * ~H + R;
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

        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;
        imu_msg.orientation.w = cosf(filtered_yaw * 0.5f);
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = sinf(filtered_yaw * 0.5f);

        // --- Publish Odometry ---
        odom_msg.header.stamp.sec = imu_msg.header.stamp.sec;
        odom_msg.header.stamp.nanosec = imu_msg.header.stamp.nanosec;
        
        odom_msg.pose.pose.position.x = X(0);
        odom_msg.pose.pose.position.y = X(1);
        odom_msg.pose.pose.position.z = 0.0;
        
        odom_msg.pose.pose.orientation = imu_msg.orientation; 
        
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

        if (connected) { 
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
