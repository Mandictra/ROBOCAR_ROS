#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#define LEFT_EN   4   // ENA -> PWM
#define LEFT_IN1  5
#define LEFT_IN2  6
#define RIGHT_EN  7   // ENB -> PWM
#define RIGHT_IN1 15
#define RIGHT_IN2 16

#define MAX_LINEAR_VEL  0.5f   
#define MAX_ANGULAR_VEL 2.0f   
#define CMD_TIMEOUT_MS  500

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

volatile uint32_t last_cmd_ms = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  // FIXED: Wait 3 seconds, then automatically reboot to try connecting again!
  delay(3000);
  ESP.restart();
}

void set_motor(int en_pin, int in1_pin, int in2_pin, float norm_speed) {
  norm_speed = constrain(norm_speed, -1.0f, 1.0f);

  if (norm_speed > 0.01f) {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (norm_speed < -0.01f) {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
  }

  int pwm = (int)(fabs(norm_speed) * 255.0f);
  analogWrite(en_pin, pwm);
}

void drive_motors(float left_speed, float right_speed) {
  float left_norm  = left_speed  / MAX_LINEAR_VEL;
  float right_norm = right_speed / MAX_LINEAR_VEL;

  set_motor(LEFT_EN, LEFT_IN1, LEFT_IN2, left_norm);
  set_motor(RIGHT_EN, RIGHT_IN1, RIGHT_IN2, right_norm);
}

void twist_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;

  last_cmd_ms = millis();

  float linear_x = twist_msg->linear.x;
  float angular_z = twist_msg->angular.z;

  float left_speed  = linear_x - angular_z;
  float right_speed = linear_x + angular_z;

  drive_motors(left_speed, right_speed);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_controller", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &twist_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
    drive_motors(0, 0);
  }
}