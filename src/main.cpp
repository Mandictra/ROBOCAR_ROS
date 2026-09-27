#include <Arduino.h>
#include <SPI.h> // Required to prevent Adafruit BusIO compilation errors
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <AccelStepper.h>

#include "pins.h" 

// ---------- Hardware Instances ----------
Servo pan_servo, dump_servo;
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// AccelStepper requires the swapped pin order for the 28BYJ-48 (IN1, IN3, IN2, IN4)
AccelStepper conveyorStepper(AccelStepper::FULL4WIRE, CONV_IN1, CONV_IN3, CONV_IN2, CONV_IN4);

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

rcl_publisher_t enc_pub;
std_msgs__msg__Int32MultiArray enc_msg;
int32_t enc_data[4];

rcl_publisher_t color_pub;
geometry_msgs__msg__Vector3 color_msg;

// ---------- State Variables ----------
volatile uint32_t last_cmd_ms = 0;
float max_linear_vel = 0.5;
bool sensor_active = false; 
float current_conveyor_cmd = 0.0; // Tracks target speed for non-blocking stepper

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ---------- Motor Control ----------
void set_motor(int in1, int in2, float norm_speed) {
  norm_speed = constrain(norm_speed, -1.0f, 1.0f);
  int pwm = (int)(fabs(norm_speed) * 250.0f); 

  if (norm_speed > 0.01f) {
    digitalWrite(in2, LOW); 
    analogWrite(in1, pwm);
  } else if (norm_speed < -0.01f) {
    digitalWrite(in1, LOW); 
    analogWrite(in2, pwm);
  } else {
    // CRITICAL FIX: Explicitly set PWM duty cycle to 0 to stop the motor. 
    // digitalWrite(LOW) is ignored by the ESP32 once a pin is in PWM mode.
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void drive_motors(float left_speed, float right_speed) {
  float left_norm  = constrain(left_speed / max_linear_vel, -1.0, 1.0);
  float right_norm = constrain(right_speed / max_linear_vel, -1.0, 1.0);
  
  set_motor(FL_IN1, FL_IN2, left_norm);
  set_motor(RL_IN1, RL_IN2, left_norm);
  set_motor(FR_IN1, FR_IN2, right_norm);
  set_motor(RR_IN1, RR_IN2, right_norm);
}

// ---------- ROS 2 Callbacks ----------
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_ms = millis();
  float left_speed  = msg->linear.x - msg->angular.z;
  float right_speed = msg->linear.x + msg->angular.z;
  drive_motors(left_speed, right_speed);
}

void pan_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  pan_servo.write((int)constrain(msg->x, PAN_MIN, PAN_MAX));
}

void dump_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  dump_servo.write((int)constrain(msg->data, 5, 175));
}

void conveyor_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  current_conveyor_cmd = msg->data;
  conveyorStepper.setSpeed(current_conveyor_cmd);
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // 1. Servo Initialization 
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  pan_servo.setPeriodHertz(50); 
  dump_servo.setPeriodHertz(50);
  pan_servo.attach(PAN_PIN, 500, 2400);
  dump_servo.attach(DUMP_PIN, 500, 2400);
  pan_servo.write(90); 
  dump_servo.write(0);

  // 2. Motor Pins & Stepper Config
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  
  drive_motors(0, 0);
  
  conveyorStepper.setMaxSpeed(1000.0); 

  // 3. PCNT Hardware Encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);
  
  enc_msg.data.capacity = 4;
  enc_msg.data.size = 4;
  enc_msg.data.data = enc_data;
 
  // 4. I2C Color Sensor Safe Init
  Wire.begin(I2C_SDA, I2C_SCL);
  if (tcs.begin()) {
    sensor_active = true;
  } else {
    sensor_active = false;
  }

  delay(2000);

  while(rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { delay(100); }

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "base_controller", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(&enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "encoders"));
  RCCHECK(rclc_publisher_init_default(&color_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "trash_color"));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&pan_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "pan_tilt_cmd"));
  RCCHECK(rclc_subscription_init_default(&dump_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "dump_cmd"));
  RCCHECK(rclc_subscription_init_default(&conv_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "conveyor_cmd"));

  // Executor (Increased capacity to 4 for the new conveyor subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator)); 
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pan_sub, &pan_msg, &pan_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &dump_sub, &dump_msg, &dump_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &conv_sub, &conv_msg, &conveyor_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10))); 
  
  uint32_t current_time = millis();
  
  // Non-blocking Stepper Execution
  if (abs(current_conveyor_cmd) > 0.1) {
    conveyorStepper.runSpeed();
  } else {
    conveyorStepper.disableOutputs(); 
  }

  // Publish encoder telemetry at 20Hz (every 50ms)
  static uint32_t last_enc_pub = 0;
  if (current_time - last_enc_pub > 50) {
    enc_msg.data.data[0] = (int32_t)enc_fl.getCount();
    enc_msg.data.data[1] = (int32_t)enc_rl.getCount();
    enc_msg.data.data[2] = (int32_t)enc_fr.getCount();
    enc_msg.data.data[3] = (int32_t)enc_rr.getCount();
    
    (void)rcl_publish(&enc_pub, &enc_msg, NULL);
    last_enc_pub = current_time;
  }
  
  // Publish color sensor data at 10Hz (every 100ms) only if active
  static uint32_t last_color_pub = 0;
  if (current_time - last_color_pub > 100) {
    if (sensor_active) {
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);
      color_msg.x = r; color_msg.y = g; color_msg.z = b;
      (void)rcl_publish(&color_pub, &color_msg, NULL);
    }
    last_color_pub = current_time;
  }
  
  
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
