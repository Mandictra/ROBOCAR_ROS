#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/point.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ---------- DRV8833 Motor Pins ----------
#define LEFT_IN1 4
#define LEFT_IN2 5
#define RIGHT_IN1 6
#define RIGHT_IN2 7

// ---------- Encoder Pins ----------
#define ENC_FL_A 10
#define ENC_FL_B 11
#define ENC_RL_A 12
#define ENC_RL_B 13
#define ENC_FR_A 14
#define ENC_FR_B 15
#define ENC_RR_A 16
#define ENC_RR_B 17

// ---------- Actuator Pins ----------
#define PAN_PIN   18
#define TILT_PIN  19
#define DUMP_PIN  20

#define CONV_IN1  8
#define CONV_IN2  9
#define CONV_IN3  21
#define CONV_IN4  38

// ---------- Sensor & Limits ----------
#define I2C_SDA 41
#define I2C_SCL 42

#define PAN_MIN   0
#define PAN_MAX   180
#define TILT_MIN  30
#define TILT_MAX  150
#define CMD_TIMEOUT_MS  500

// ---------- Hardware Instances ----------
ESP32Encoder enc_fl, enc_rl, enc_fr, enc_rr;
Servo pan_servo, tilt_servo, dump_servo;
AccelStepper conveyor(AccelStepper::HALF4WIRE, CONV_IN1, CONV_IN3, CONV_IN2, CONV_IN4);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ---------- micro-ROS Entities ----------
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rclc_parameter_server_t param_server;

rcl_subscription_t twist_sub;
geometry_msgs__msg__Twist twist_msg;

rcl_subscription_t pan_tilt_sub;
geometry_msgs__msg__Point pan_tilt_msg;

rcl_subscription_t dump_sub;
std_msgs__msg__Float32 dump_msg;

rcl_subscription_t conveyor_sub;
std_msgs__msg__Float32 conveyor_msg;

rcl_publisher_t enc_pub;
std_msgs__msg__Int32MultiArray enc_msg;
int32_t enc_data[4];

rcl_publisher_t color_pub;
geometry_msgs__msg__Vector3 color_msg;

// ---------- State Variables ----------
volatile uint32_t last_cmd_ms = 0;
float max_linear_vel = 0.5;
float conv_max_speed = 800.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  delay(3000);
  ESP.restart();
}

// ---------- Motor Control ----------
void set_motor(int in1, int in2, float norm_speed) {
  norm_speed = constrain(norm_speed, -1.0f, 1.0f);
  int pwm = (int)(fabs(norm_speed) * 255.0f);

  if (norm_speed > 0.01f) {
    analogWrite(in1, pwm);
    analogWrite(in2, 0);
  } else if (norm_speed < -0.01f) {
    analogWrite(in1, 0);
    analogWrite(in2, pwm);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void drive_motors(float left_speed, float right_speed) {
  float left_norm  = constrain(left_speed / max_linear_vel, -1.0, 1.0);
  float right_norm = constrain(right_speed / max_linear_vel, -1.0, 1.0);
  set_motor(LEFT_IN1, LEFT_IN2, left_norm);
  set_motor(RIGHT_IN1, RIGHT_IN2, right_norm);
}

// ---------- ROS 2 Callbacks ----------
void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_ms = millis();
  float left_speed  = msg->linear.x - msg->angular.z;
  float right_speed = msg->linear.x + msg->angular.z;
  drive_motors(left_speed, right_speed);
}

void pan_tilt_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  pan_servo.write((int)constrain(msg->x, PAN_MIN, PAN_MAX));
  tilt_servo.write((int)constrain(msg->y, TILT_MIN, TILT_MAX));
}

void dump_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  dump_servo.write((int)constrain(msg->data, 0, 180));
}

void conveyor_callback(const void * msgin) {
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  conveyor.setSpeed(constrain(msg->data, -conv_max_speed, conv_max_speed));
}

bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context) {
  (void)context; (void)old_param;
  if (new_param == NULL) return false;
  if (strcmp(new_param->name.data, "max_speed") == 0) {
    max_linear_vel = new_param->value.double_value;
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  // 1. DRV8833 Pins
  pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT);
  drive_motors(0, 0);

  // 2. PCNT Encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);
  enc_msg.data.capacity = 4;
  enc_msg.data.size = 4;
  enc_msg.data.data = enc_data;

  // 3. Servos & Stepper
  pan_servo.setPeriodHertz(50); tilt_servo.setPeriodHertz(50); dump_servo.setPeriodHertz(50);
  pan_servo.attach(PAN_PIN, 500, 2400);
  tilt_servo.attach(TILT_PIN, 500, 2400);
  dump_servo.attach(DUMP_PIN, 500, 2400);
  pan_servo.write(90); tilt_servo.write(90); dump_servo.write(0);
  
  conveyor.setMaxSpeed(conv_max_speed);

  // 4. I2C Color Sensor
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!tcs.begin()) {
    Serial.println("TCS34725 not found!");
  }

  delay(2000);

  // 5. ROS 2 Initialization[cite: 2]
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_base_controller", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(&enc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "encoders"));
  RCCHECK(rclc_publisher_init_default(&color_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "trash_color"));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&twist_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&pan_tilt_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point), "pan_tilt_cmd"));
  RCCHECK(rclc_subscription_init_default(&dump_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "dump_cmd"));
  RCCHECK(rclc_subscription_init_default(&conveyor_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "conveyor_cmd"));

  // Parameter Server
  const rclc_parameter_options_t param_opts = { .notify_changed_over_dds = true, .max_params = 1 };
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_opts));
  RCCHECK(rclc_add_parameter(&param_server, "max_speed", RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_parameter_set_double(&param_server, "max_speed", 0.5));

  // Executor (4 Subs + 1 Param = 5 handles)
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pan_tilt_sub, &pan_tilt_msg, &pan_tilt_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &dump_sub, &dump_msg, &dump_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &conveyor_sub, &conveyor_msg, &conveyor_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, 0)); // 0 Timeout to avoid blocking the stepper[cite: 2]

  uint32_t current_time = millis();

  // Publish encoder data (Every 50ms)[cite: 2]
  static uint32_t last_enc_pub = 0;
  if (current_time - last_enc_pub > 50) {
    enc_msg.data.data[0] = (int32_t)enc_fl.getCount();
    enc_msg.data.data[1] = (int32_t)enc_rl.getCount();
    enc_msg.data.data[2] = (int32_t)enc_fr.getCount();
    enc_msg.data.data[3] = (int32_t)enc_rr.getCount();
    rcl_publish(&enc_pub, &enc_msg, NULL);
    last_enc_pub = current_time;
  }

  // Publish color sensor data (Every 100ms)[cite: 2]
  static uint32_t last_color_pub = 0;
  if (current_time - last_color_pub > 100) {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    color_msg.x = r; color_msg.y = g; color_msg.z = b;
    rcl_publish(&color_pub, &color_msg, NULL);
    last_color_pub = current_time;
  }

  // Safety switch (Dead Man's Switch) for drive motors[cite: 2]
  if (current_time - last_cmd_ms > CMD_TIMEOUT_MS) {
    drive_motors(0, 0);
  }

  // Keep the stepper motor running[cite: 2]
  conveyor.runSpeed();
}