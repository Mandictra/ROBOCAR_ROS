// motor_id_test.cpp - guided motor + encoder identification.
// ROBOT ON BLOCKS, WHEELS OFF THE GROUND. Serial monitor at 115200, send with ENTER.
// Spins each motor alone for 1 s using exactly the drive law in main.cpp
// (command +0.3 x MOTOR_DIR_xx, i.e. what "forward on this wheel" does in firmware),
// prints the delta of all four encoders, and asks which physical wheel moved and which way.
// Copy every line starting with RESULT and paste it back.
//
// platformio.ini:
//   [env:motor_test]
//   extends = env:esp32-s3-devkitc-1
//   build_src_filter = -<*> +<motor_id_test.cpp>

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "pins.h"

#define MIN_PWM 150
#define MAX_PWM 250
#define TEST_CMD 0.3f

ESP32Encoder enc[4];   // index: 0=FL 1=FR 2=RL 3=RR, labels as in pins.h
const char *NAME[4] = {"FL", "FR", "RL", "RR"};
const int IN1[4]  = {FL_IN1, FR_IN1, RL_IN1, RR_IN1};
const int IN2[4]  = {FL_IN2, FR_IN2, RL_IN2, RR_IN2};
const int DIRC[4] = {MOTOR_DIR_FL, MOTOR_DIR_FR, MOTOR_DIR_RL, MOTOR_DIR_RR};

static void setMotor(int i, float s) {
  s = constrain(s * DIRC[i], -1.0f, 1.0f);
  int pwm = fabsf(s) > 0.01f ? MIN_PWM + (int)(fabsf(s) * (MAX_PWM - MIN_PWM)) : 0;
  if (pwm == 0)      { digitalWrite(IN2[i], LOW);  analogWrite(IN1[i], 0); }
  else if (s > 0)    { digitalWrite(IN2[i], LOW);  analogWrite(IN1[i], pwm); }
  else               { digitalWrite(IN2[i], HIGH); analogWrite(IN1[i], 255 - pwm); }
}

static void waitEnter() {
  Serial.println(">> press ENTER when ready");
  while (Serial.available()) Serial.read();
  while (!Serial.available()) delay(10);
  delay(50);
  while (Serial.available()) Serial.read();
}

static String readLine() {
  while (Serial.available()) Serial.read();
  String s;
  while (true) {
    while (!Serial.available()) delay(10);
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (s.length()) return s; }
    else s += c;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  analogWriteFrequency(20000);
  for (int i = 0; i < 4; i++) {
    pinMode(IN1[i], OUTPUT); pinMode(IN2[i], OUTPUT);
    setMotor(i, 0);
  }
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc[0].attachHalfQuad(ENC_FL_A, ENC_FL_B);
  enc[1].attachHalfQuad(ENC_FR_A, ENC_FR_B);
  enc[2].attachHalfQuad(ENC_RL_A, ENC_RL_B);
  enc[3].attachHalfQuad(ENC_RR_A, ENC_RR_B);

  Serial.println("\nMotor/encoder ID test - 4 motors, about 2 minutes");
  Serial.println("Wheel names = the ROBOT'S OWN left/right (stand behind it, look the way it drives).");

  for (int i = 0; i < 4; i++) {
    Serial.printf("\n==== MOTOR %s (pins.h label) ====\n", NAME[i]);
    Serial.println("It will spin for 1 s. Watch which wheel moves and which way (forward/backward).");
    waitEnter();

    int64_t c0[4];
    for (int k = 0; k < 4; k++) c0[k] = enc[k].getCount();
    setMotor(i, TEST_CMD);
    delay(1000);
    setMotor(i, 0);
    delay(300);
    long d[4];
    for (int k = 0; k < 4; k++) d[k] = (long)(enc[k].getCount() - c0[k]);

    Serial.printf("RESULT M%s encoder deltas  FL=%ld FR=%ld RL=%ld RR=%ld\n", NAME[i], d[0], d[1], d[2], d[3]);
    Serial.println("Which wheel moved and which way? Type e.g.  FR F   (wheel FL/FR/RL/RR, then F or B), ENTER:");
    String ans = readLine();
    ans.toUpperCase();
    Serial.printf("RESULT M%s observed = %s\n", NAME[i], ans.c_str());
  }

  Serial.println("\n==== ALL DONE ====");
  Serial.println("Copy every line that starts with RESULT and paste it back.");
}

void loop() { delay(100); }