#include <Arduino.h>
#include <ESP32Encoder.h>
#include "pins.h"

// Hardware Instances
ESP32Encoder enc_fl, enc_fr, enc_rl, enc_rr;

// Constants
const float TICKS_PER_REV = 1000.0f;
const int SAMPLE_TIME_MS = 50; // Updated to match autotune dt

// State Variables
unsigned long last_time = 0;
int32_t last_fl = 0, last_fr = 0, last_rl = 0, last_rr = 0;
float target_rpm = 0.0f; // Now commands target RPM instead of raw PWM

// PID Class
class SimplePID {
public:
  float kp, ki, kd;
  float ff_gain;    
  float ff_offset;  
  float integral = 0.0f;
  float prev_error = 0.0f;

  SimplePID(float p, float i, float d, float f, float b) : kp(p), ki(i), kd(d), ff_gain(f), ff_offset(b) {}

  int compute(float tgt, float current, float dt) {
    if (tgt == 0.0f) {
      integral = 0.0f;
      prev_error = 0.0f;
      return 0;
    }

    float error = tgt - current;
    integral += error * dt;
    
    // Anti-windup
    if (ki > 0.0f) {
      float lim = 255.0f / ki;
      integral = constrain(integral, -lim, lim);
    } else {
      integral = 0.0f;
    }

    float derivative = (dt > 0.0f) ? (error - prev_error) / dt : 0.0f;
    prev_error = error;

    float ff = tgt * ff_gain + (tgt > 0.0f ? ff_offset : -ff_offset);
    float output = ff + (kp * error) + (ki * integral) + (kd * derivative);
    return (int)constrain(output, -255.0f, 255.0f);
  }
};

// Initialize PID controllers with tuned_gains.json values
// Format: SimplePID(P, I, D, F, B)
SimplePID pid_fl(0.18, 5.97, 0.0, 0.967, 140.8);
SimplePID pid_fr(0.31, 10.38, 0.0, 2.043, 10.9);
SimplePID pid_rl(0.38, 10.75, 0.0, 2.117, 8.1);
SimplePID pid_rr(0.15, 4.96, 0.0, 0.976, 140.7);

// Simplified motor driver
void set_raw_motor(int in1, int in2, int pwm) {
  if (pwm == 0) {
    digitalWrite(in2, LOW); analogWrite(in1, 0);
  } else if (pwm > 0) {
    digitalWrite(in2, LOW); analogWrite(in1, pwm);
  } else {
    digitalWrite(in2, HIGH); analogWrite(in1, 255 + pwm); // pwm is negative here
  }
}

void setup() {
  Serial.begin(115200);
  analogWriteFrequency(20000);

  // Initialize Motor Pins
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);

  // Initialize Encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_fl.attachHalfQuad(ENC_FL_A, ENC_FL_B);
  enc_fr.attachHalfQuad(ENC_FR_A, ENC_FR_B);
  enc_rl.attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc_rr.attachHalfQuad(ENC_RR_A, ENC_RR_B);

  Serial.println("Send a Target RPM value to spin motors. Send 0 to stop.");
}

void loop() {
  // 1. Read Serial Command for Target RPM
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    target_rpm = input.toFloat();
    target_rpm = constrain(target_rpm, -70.0f, 70.0f); // Constrain to safe RPM bounds
  }

  unsigned long current_time = millis();
  if (current_time - last_time >= SAMPLE_TIME_MS) {
    float dt = (current_time - last_time) / 1000.0f;
    
    int32_t current_fl = enc_fl.getCount();
    int32_t current_fr = enc_fr.getCount();
    int32_t current_rl = enc_rl.getCount();
    int32_t current_rr = enc_rr.getCount();
    
    // Formula: (Delta Ticks / Ticks per Rev) * (60 seconds / Delta Time) * Direction Fix
    float rpm_fl = ((current_fl - last_fl) / TICKS_PER_REV) * (60.0f / dt) * MOTOR_DIR_FL;
    float rpm_fr = ((current_fr - last_fr) / TICKS_PER_REV) * (60.0f / dt) * MOTOR_DIR_FR;
    float rpm_rl = ((current_rl - last_rl) / TICKS_PER_REV) * (60.0f / dt) * MOTOR_DIR_RL;
    float rpm_rr = ((current_rr - last_rr) / TICKS_PER_REV) * (60.0f / dt) * MOTOR_DIR_RR;
    
    // 2. Compute PID outputs based on actual RPM
    int pwm_fl = pid_fl.compute(target_rpm, rpm_fl, dt);
    int pwm_fr = pid_fr.compute(target_rpm, rpm_fr, dt);
    int pwm_rl = pid_rl.compute(target_rpm, rpm_rl, dt);
    int pwm_rr = pid_rr.compute(target_rpm, rpm_rr, dt);

    // 3. Drive Motors (applying direction correction)
    set_raw_motor(FL_IN1, FL_IN2, pwm_fl * MOTOR_DIR_FL);
    set_raw_motor(FR_IN1, FR_IN2, pwm_fr * MOTOR_DIR_FR);
    set_raw_motor(RL_IN1, RL_IN2, pwm_rl * MOTOR_DIR_RL);
    set_raw_motor(RR_IN1, RR_IN2, pwm_rr * MOTOR_DIR_RR);
    
    // Output formatted for the Arduino Serial Plotter
    Serial.print("Target_RPM:"); Serial.print(target_rpm); Serial.print(" ");
    Serial.print("FL_RPM:"); Serial.print(rpm_fl); Serial.print(" ");
    Serial.print("FR_RPM:"); Serial.print(rpm_fr); Serial.print(" ");
    Serial.print("RL_RPM:"); Serial.print(rpm_rl); Serial.print(" ");
    Serial.print("RR_RPM:"); Serial.println(rpm_rr);

    last_fl = current_fl;
    last_fr = current_fr;
    last_rl = current_rl;
    last_rr = current_rr;
    last_time = current_time;
  }
}