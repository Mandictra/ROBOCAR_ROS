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
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>

#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <AccelStepper.h>

#include "pins.h" // External file containing all hardware pin definitions 

// ---------- Hardware Instances ----------
Servo pan_servo, dump_servo;
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr; // Hardware pulse counters for wheel odometry 

// Conveyor Stepper: Configured in 4-wire mode to drive the main belt 
AccelStepper conveyorStepper(AccelStepper::FULL4WIRE, CONV_IN1, CONV_IN3, CONV_IN2, CONV_IN4);

// Standard PWM pulse limits for typical 180-degree hobby servos 
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2400

// ---------- micro-ROS Entities ----------
// Standard micro-ROS architecture requirements for node and execution management 
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Subscribers: Listen to instructions coming from the Raspberry Pi 
rcl_subscription_t twist_sub;
geometry_msgs__msg__Twist twist_msg; // Receives directional movement data 

rcl_subscription_t pan_sub;
geometry_msgs__msg__Point pan_msg;   // Receives camera pan angles 

rcl_subscription_t dump_sub;
std_msgs__msg__Float32 dump_msg;     // Receives dump bin actuation angles 

rcl_subscription_t conv_sub;
std_msgs__msg__Float32 conv_msg;     // Receives target speed for the conveyor belt 

rcl_subscription_t speed_sub;
std_msgs__msg__Float32 speed_msg;    // Receives a dynamic multiplier for max robot speed 

// Publishers: Send sensor data back up to the Raspberry Pi 
rcl_publisher_t enc_pub;
std_msgs__msg__Int32MultiArray enc_msg; // Packages 4 encoder values into one message 
int32_t enc_data[4];

rcl_publisher_t ir_pub;
std_msgs__msg__Bool ir_msg;             // Sends simple True/False object detection status 

// ---------- State Variables ----------
volatile uint32_t last_cmd_ms = 0;       // Tracks time of last movement command for safety timeout 
float max_linear_vel = 0.5;              // Global speed limit multiplier 
float current_conveyor_cmd = 0.0;        // Current requested speed of the stepper motor 

uint32_t last_dump_cmd_ms = 0;           // Tracks when the servo was last moved 
bool dump_servo_attached = true;         // Flag to manage servo power state 

// Error checking macros: Halt execution if ROS fails to initialize 
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ---------- Motor Control ----------

// Deadband limits: Below 150 PWM, the motors lack the torque to move 
#define MOTOR_MIN_PWM 150
#define MOTOR_MAX_PWM 250

/**
 * Translates a normalized speed (-1.0 to 1.0) into physical PWM signals.
 * Applies the deadband offset so even small speed requests provide enough power to move .
 */
void set_motor(int in1, int in2, float norm_speed) {
  norm_speed = constrain(norm_speed, -1.0f, 1.0f);
  int pwm = 0;
  
  // Calculate PWM only if speed is above the zero-threshold 
  if (fabs(norm_speed) > 0.01f) {
    pwm = MOTOR_MIN_PWM + (int)(fabs(norm_speed) * (MOTOR_MAX_PWM - MOTOR_MIN_PWM));
  }

  // Directional logic using H-bridge pins 
  if (norm_speed > 0.01f) {
    analogWrite(in2, 0);
    analogWrite(in1, pwm);
  } else if (norm_speed < -0.01f) {
    analogWrite(in1, 0);
    analogWrite(in2, pwm);
  } else {
    // Hard stop 
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

/**
 * Maps left and right track speeds to the 4 physical drive motors.
 * Scales the requested speed by the current global max speed limit .
 */
void drive_motors(float left_speed, float right_speed) {
  float left_norm  = constrain(left_speed, -1.0f, 1.0f) * max_linear_vel;
  float right_norm = constrain(right_speed, -1.0f, 1.0f) * max_linear_vel;

  set_motor(FL_IN1, FL_IN2, left_norm);
  set_motor(RL_IN1, RL_IN2, left_norm);
  set_motor(FR_IN1, FR_IN2, right_norm);
  set_motor(RR_IN1, RR_IN2, right_norm);
}

// ---------- ROS 2 Callbacks ----------

/**
 * Triggered when Nav2 or the joystick sends a movement command.
 * Converts Twist kinematics (linear X, angular Z) to skid-steer left/right wheel speeds .
 */
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_ms = millis(); // Reset safety timeout 
  float left_speed  = msg->linear.x - msg->angular.z;
  float right_speed = msg->linear.x + msg->angular.z;
  drive_motors(left_speed, right_speed);
}

/**
 * Updates camera pan servo angle (constrained to safe physical limits) .
 */
void pan_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  pan_servo.write((int)constrain(msg->x, PAN_MIN, PAN_MAX));
}

/**
 * Updates dump bin servo angle. Re-attaches the servo to a PWM timer if it was disabled .
 */
void dump_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  if (!dump_servo_attached) {
    dump_servo.attach(DUMP_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    dump_servo_attached = true;
  }
  dump_servo.write((int)constrain(msg->data, 5, 175));
  last_dump_cmd_ms = millis();
}

/**
 * Updates target speed for the conveyor stepper motor .
 */
void conveyor_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  current_conveyor_cmd = msg->data;
  conveyorStepper.setSpeed(current_conveyor_cmd);
}

/**
 * Dynamically adjusts the robot's top speed (capped between 10% and 100%) .
 */
void speed_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  max_linear_vel = constrain(msg->data, 0.1f, 1.0f);
}

void setup() {
  Serial.begin(115200);

  // Push PWM frequency to 20kHz to eliminate audible motor whining 
  analogWriteFrequency(20000);

  set_microros_serial_transports(Serial);

  // 1. Servo Initialization
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  pan_servo.setPeriodHertz(50);
  dump_servo.setPeriodHertz(50);
  pan_servo.attach(PAN_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  dump_servo.attach(DUMP_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  pan_servo.write(90); // Center camera 
  dump_servo.write(5); // Retract bin 
  last_dump_cmd_ms = millis();

  // 2. Motor Pins & Stepper Config
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  drive_motors(0, 0); // Ensure stopped at boot 
  conveyorStepper.setMaxSpeed(700.0);

  // 3. PCNT Hardware Encoders (Uses internal ESP32 pullups for cleaner signal) 
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);

  // Pre-allocate memory for the encoder message array to prevent crashes 
  enc_msg.data.capacity = 4;
  enc_msg.data.size = 4;
  enc_msg.data.data = enc_data;

  // 4. IR Sensor Initialization
  // Uses INPUT mode. Switch to INPUT_PULLUP if sensor reads floating values when empty .
  pinMode(IR_SENSOR_PIN, INPUT);

  delay(2000); // Allow hardware to stabilize 

  // Wait for the micro-ROS agent on the Pi to connect 
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { delay(100); }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_controller", "", &support));

  // Initialize Publishers 
  RCCHECK(rclc_publisher_init_default(&enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "encoders"));
  RCCHECK(rclc_publisher_init_default(&ir_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "ir_trigger"));

  // Initialize Subscribers 
  RCCHECK(rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&pan_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "pan_tilt_cmd"));
  RCCHECK(rclc_subscription_init_default(&dump_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "dump_cmd"));
  RCCHECK(rclc_subscription_init_default(&conv_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "conveyor_cmd"));
  RCCHECK(rclc_subscription_init_default(&speed_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "max_speed_cmd"));

  // Executor setup: Must match the number of active subscriptions (5) 
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pan_sub, &pan_msg, &pan_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &dump_sub, &dump_msg, &dump_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &conv_sub, &conv_msg, &conveyor_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &speed_sub, &speed_msg, &speed_callback, ON_NEW_DATA));
}

void loop() {
  // 1. Process incoming ROS messages quickly without blocking 
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));

  uint32_t current_time = millis();

  // 2. Dynamic Detach Hack: 
  // Cheap servos buzz and draw heavy current when holding position. 
  // This detaches the PWM signal 1 second after moving to save power and stop jitter .
  if (dump_servo_attached && (current_time - last_dump_cmd_ms > 1000)) {
    dump_servo.detach();
    pinMode(DUMP_PIN, OUTPUT);
    digitalWrite(DUMP_PIN, LOW);
    dump_servo_attached = false;
  }

  // 3. Unblocked Stepper Execution: 
  // Steps the motor incrementally if required, keeping the loop fast .
  if (abs(current_conveyor_cmd) > 0.1) {
    conveyorStepper.enableOutputs();
    conveyorStepper.runSpeed();
  } else {
    conveyorStepper.disableOutputs(); // Drops coil power to save battery when stopped 
  }

  // 4. Publish encoder telemetry at 20Hz (every 50ms)
  static uint32_t last_enc_pub = 0;
  if (current_time - last_enc_pub > 50) {
    enc_msg.data.data[0] = (int32_t)enc_fl.getCount();
    enc_msg.data.data[1] = (int32_t)enc_rl.getCount();
    enc_msg.data.data[2] = (int32_t)enc_fr.getCount();
    enc_msg.data.data[3] = (int32_t)enc_rr.getCount();

    (void)rcl_publish(&enc_pub, &enc_msg, NULL);
    last_enc_pub = current_time;
  }

  // 5. Publish IR Sensor Data at 10Hz (every 100ms)
  static uint32_t last_ir_pub = 0;
  if (current_time - last_ir_pub > 100) {
    ir_msg.data = (digitalRead(IR_SENSOR_PIN) == LOW); // LOW indicates object detected
    (void)rcl_publish(&ir_pub, &ir_msg, NULL);
    last_ir_pub = current_time;
  }

  // 6. Command Timeout Failsafe:
  // Automatically cuts power to the drive wheels if connection is lost or 
  // no commands arrive within CMD_TIMEOUT_MS to prevent runaways.
  static bool is_stopped = false;
  if (current_time - last_cmd_ms > CMD_TIMEOUT_MS) {
    if (!is_stopped) {
      drive_motors(0, 0);
      is_stopped = true;
    }
  } else {
    is_stopped = false;
  }
}