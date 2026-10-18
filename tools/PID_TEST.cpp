#include <Arduino.h>
#include <ESP32Encoder.h>
#include "pins.h"

// Index order everywhere: 0=FL('1'), 1=FR('2'), 2=RL('3'), 3=RR('4'). Selector 'A' = all.
ESP32Encoder enc[4];

const float TICKS_PER_REV = 748.0f;
const int SAMPLE_TIME_MS = 50;  // must match SAMPLE_DT in motor_calibrate.py

const int IN1[4] = {FL_IN1, FR_IN1, RL_IN1, RR_IN1};
const int IN2[4] = {FL_IN2, FR_IN2, RL_IN2, RR_IN2};
const int ENC_A[4] = {ENC_FL_A, ENC_FR_A, ENC_RL_A, ENC_RR_A};
const int ENC_B[4] = {ENC_FL_B, ENC_FR_B, ENC_RL_B, ENC_RR_B};
const int DIR[4] = {MOTOR_DIR_FL, MOTOR_DIR_FR, MOTOR_DIR_RL, MOTOR_DIR_RR};

unsigned long last_time = 0;
int32_t last_count[4] = {0, 0, 0, 0};
float target[4] = {0, 0, 0, 0};
float target_rpm = 0.0f;  // last commanded target, only for the dashboard line

class SimplePID {
public:
  float kp, ki, kd;
  float ff_gain = 3.4f;    // PWM per RPM  (= 1/K from autotune)
  float ff_offset = 0.0f;  // PWM needed to overcome deadband (pwm0 from autotune)
  // Separate gain set used when the target is negative (reverse). Commands p,i,d,f,b (lower case).
  float kp_r, ki_r, kd_r;
  float ff_gain_r = 3.4f;
  float ff_offset_r = 0.0f;
  bool last_rev = false;
  bool open_loop = false;  // autotune: bypass PID, output open_pwm directly
  int open_pwm = 0;
  float integral = 0.0f;
  float prev_error = 0.0f;

  SimplePID(float p, float i, float d) : kp(p), ki(i), kd(d), kp_r(p), ki_r(i), kd_r(d) {}

  int compute(float tgt, float current, float dt) {
    if (open_loop) {
      integral = 0.0f;
      prev_error = 0.0f;
      return open_pwm;
    }
    if (tgt == 0.0f) {
      integral = 0.0f;
      prev_error = 0.0f;
      return 0;
    }

    // Pick the gain set for the direction of the target.
    bool rev = tgt < 0.0f;
    float KP = rev ? kp_r : kp, KI = rev ? ki_r : ki, KD = rev ? kd_r : kd;
    float FFG = rev ? ff_gain_r : ff_gain, FFO = rev ? ff_offset_r : ff_offset;
    if (rev != last_rev) {            // direction change: start the new loop clean
      integral = 0.0f;
      prev_error = 0.0f;
      last_rev = rev;
    }

    float error = tgt - current;
    integral += error * dt;
    // Clamp so KI*integral can never exceed the PWM range (a fixed 150 allowed 750 PWM at ki=5)
    if (KI > 0.0f) {
      float lim = 255.0f / KI;
      integral = constrain(integral, -lim, lim);
    } else {
      integral = 0.0f;
    }

    float derivative = (dt > 0.0f) ? (error - prev_error) / dt : 0.0f;
    prev_error = error;

    float ff = tgt * FFG + (tgt > 0.0f ? FFO : -FFO);
    float output = ff + (KP * error) + (KI * integral) + (KD * derivative);
    return (int)constrain(output, -255.0f, 255.0f);
  }
};

SimplePID pid[4] = {SimplePID(2.0, 5.0, 0.0), SimplePID(2.0, 5.0, 0.0),
                    SimplePID(2.0, 5.0, 0.0), SimplePID(2.0, 5.0, 0.0)};

void set_raw_motor(int in1, int in2, int pwm) {
  if (pwm == 0) {
    digitalWrite(in2, LOW); analogWrite(in1, 0);
  } else if (pwm > 0) {
    digitalWrite(in2, LOW); analogWrite(in1, pwm);
  } else {
    digitalWrite(in2, HIGH); analogWrite(in1, 255 + pwm);
  }
}

// Commands: <cmd><sel><value>\n   sel = A,1,2,3,4
//  T rpm target (closed loop)   P/I/D gains   F feedforward PWM-per-RPM
//  B feedforward offset PWM     M raw open-loop PWM (autotune, negative = reverse); any T returns to closed loop
//  p i d f b = the same five gains for the reverse direction (target < 0)
void handle_serial() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');  // bounded by Serial.setTimeout() below
  input.trim();
  if (input.length() <= 2) return;

  char cmd = input.charAt(0);
  char sel = input.charAt(1);
  float val = input.substring(2).toFloat();

  for (int i = 0; i < 4; i++) {
    if (sel != 'A' && sel != ('1' + i)) continue;
    switch (cmd) {
      case 'T':
        target[i] = constrain(val, -120.0f, 120.0f);
        target_rpm = target[i];
        pid[i].open_loop = false;
        break;
      case 'P': pid[i].kp = val; break;
      case 'I': pid[i].ki = val; break;
      case 'D': pid[i].kd = val; break;
      case 'F': pid[i].ff_gain = val; break;
      case 'B': pid[i].ff_offset = val; break;
      case 'p': pid[i].kp_r = val; break;          // reverse set (target < 0)
      case 'i': pid[i].ki_r = val; break;
      case 'd': pid[i].kd_r = val; break;
      case 'f': pid[i].ff_gain_r = val; break;
      case 'b': pid[i].ff_offset_r = val; break;
      case 'M':
        pid[i].open_loop = true;
        pid[i].open_pwm = constrain((int)val, -255, 255);
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);  // default 1000 ms would stall the control loop on a partial line
  analogWriteFrequency(20000);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  for (int i = 0; i < 4; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    enc[i].attachHalfQuad(ENC_A[i], ENC_B[i]);
  }

  Serial.println("Cmds: T/P/I/D/F/B/M + sel(A,1-4) + value. e.g. TA30");
}

void loop() {
  handle_serial();

  unsigned long now = millis();
  if (now - last_time < SAMPLE_TIME_MS) return;
  float dt = (now - last_time) / 1000.0f;
  last_time = now;

  float rpm[4];
  for (int i = 0; i < 4; i++) {
    int32_t c = enc[i].getCount();
    // Resolution is 60/(TICKS_PER_REV*dt) = 1.2 rpm per tick at 50 ms, so Kd will mostly amplify quantisation noise.
    rpm[i] = ((c - last_count[i]) / TICKS_PER_REV) * (60.0f / dt) * DIR[i];
    last_count[i] = c;

    int pwm = pid[i].compute(target[i], rpm[i], dt);
    set_raw_motor(IN1[i], IN2[i], pwm * DIR[i]);
  }

  Serial.print("Target:"); Serial.print(target_rpm); Serial.print(" ");
  Serial.print("FL:"); Serial.print(rpm[0]); Serial.print(" ");
  Serial.print("FR:"); Serial.print(rpm[1]); Serial.print(" ");
  Serial.print("RL:"); Serial.print(rpm[2]); Serial.print(" ");
  Serial.print("RR:"); Serial.println(rpm[3]);
}