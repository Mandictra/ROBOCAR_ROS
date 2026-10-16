// imu_guided_test.cpp - guided IMU test. The terminal tells you what to do and when,
// then prints RESULT lines. Copy every line starting with RESULT and paste it back.
//
// Flash INSTEAD of main.cpp. No motor code: nothing drives, you move the robot by hand.
// Needs pins.h, MPU6050_Custom.h/.cpp from your project. Serial monitor at 115200.
//
// platformio.ini (put this file in src/, replace the env name in "extends"):
//   [env:imu_test]
//   extends = env:<your existing env>
//   build_src_filter = -<*> +<imu_guided_test.cpp> +<MPU6050_Custom.cpp>

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "pins.h"
#include "MPU6050_Custom.h"

#define SAMPLE_US 20000UL   // 50 Hz
#define DUMP_RAW  0         // 1 = also stream every sample as CSV (long output)

MPU6050_Custom imu;

struct Win {
  uint32_t n = 0;
  double sum[6]  = {0, 0, 0, 0, 0, 0};
  double sum2[6] = {0, 0, 0, 0, 0, 0};
  double ang[3]  = {0, 0, 0};   // integral of (gyro - bias) over the window [rad], about x, y, z
};

static double bias[3] = {0, 0, 0};   // gyro means measured in stage 1 [rad/s]
static int stageNo = 0;

// Sample at 50 Hz for ms milliseconds. Gyro integrals use the real timestamps.
static void collect(uint32_t ms, Win &w) {
  uint32_t t0 = millis();
  uint32_t prev = micros();
  while (millis() - t0 < ms) {
    uint32_t now = micros();
    if (now - prev < SAMPLE_US) { delay(1); continue; }
    float v[6];
    if (!imu.readData(v[0], v[1], v[2], v[3], v[4], v[5])) { delay(1); continue; }
    double dt = (now - prev) * 1e-6;
    prev = now;
    w.n++;
    for (int i = 0; i < 6; i++) { w.sum[i] += v[i]; w.sum2[i] += (double)v[i] * v[i]; }
    for (int i = 0; i < 3; i++) w.ang[i] += (v[3 + i] - bias[i]) * dt;
#if DUMP_RAW
    Serial.printf("RAW,%d,%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", stageNo, (unsigned long)millis(),
                  v[0], v[1], v[2], v[3], v[4], v[5]);
#endif
  }
}

static double mean(const Win &w, int i) { return w.n ? w.sum[i] / w.n : 0.0; }

static double sdev(const Win &w, int i) {
  if (w.n < 2) return 0.0;
  double m = mean(w, i);
  double var = w.sum2[i] / w.n - m * m;
  return var > 0 ? sqrt(var) : 0.0;
}

// + = nose up (ax goes positive when the front lifts)
static double climbDeg(const Win &w) {
  double ax = mean(w, 0), ay = mean(w, 1), az = mean(w, 2);
  return atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
}

// + = left side up (ay goes positive when the left side lifts)
static double leftUpDeg(const Win &w) {
  return atan2(mean(w, 1), mean(w, 2)) * 180.0 / PI;
}

static void waitEnter() {
  Serial.println(">> press ENTER when ready");
  while (Serial.available()) Serial.read();
  while (!Serial.available()) delay(10);
  delay(50);
  while (Serial.available()) Serial.read();
}

static void countdown() {
  Serial.print("   starting in ");
  for (int i = 3; i > 0; i--) { Serial.printf("%d ", i); delay(1000); }
  Serial.println();
}

static void phase(const char *msg, uint32_t ms, Win &w) {
  Serial.printf(">>> %s  (%lu s)\n", msg, (unsigned long)(ms / 1000));
  collect(ms, w);
}

// ---------------------------------------------------------------- stage 1
static void stage1() {
  stageNo = 1;
  Serial.println("\n==== STAGE 1/4: STILL ====");
  Serial.println("Put the robot flat on a level surface, wheels down.");
  Serial.println("Do NOT touch the robot or the table until it says DONE.");
  waitEnter();
  countdown();
  Win w;
  phase("RECORDING - hands off", 8000, w);
  Serial.println(">>> DONE");

  for (int i = 0; i < 3; i++) bias[i] = mean(w, 3 + i);
  double ax = mean(w, 0), ay = mean(w, 1), az = mean(w, 2);
  Serial.printf("RESULT S1 samples=%lu\n", (unsigned long)w.n);
  Serial.printf("RESULT S1 accel mean ax=%.3f ay=%.3f az=%.3f |a|=%.3f m/s2\n",
                ax, ay, az, sqrt(ax * ax + ay * ay + az * az));
  Serial.printf("RESULT S1 accel std  ax=%.4f ay=%.4f az=%.4f\n", sdev(w, 0), sdev(w, 1), sdev(w, 2));
  Serial.printf("RESULT S1 gyro bias  gx=%.5f gy=%.5f gz=%.5f rad/s\n", bias[0], bias[1], bias[2]);
  Serial.printf("RESULT S1 gyro std   gx=%.5f gy=%.5f gz=%.5f rad/s\n", sdev(w, 3), sdev(w, 4), sdev(w, 5));
  Serial.printf("RESULT S1 apparent climb=%.2f deg leftUp=%.2f deg\n", climbDeg(w), leftUpDeg(w));
  if (sdev(w, 3) > 0.01 || sdev(w, 4) > 0.01 || sdev(w, 5) > 0.01)
    Serial.println("WARNING: gyro too noisy - the robot was moving or vibrating. Re-run.");
}

// ---------------------------------------------------------------- stage 2
static void stage2() {
  stageNo = 2;
  Serial.println("\n==== STAGE 2/4: SPIN ====");
  Serial.println("Put a tape mark on the floor/table at the robot's nose (its start heading).");
  Serial.println("After GO: turn the robot ONE FULL TURN COUNTER-CLOCKWISE (seen from above),");
  Serial.println("flat on the surface, smoothly, taking about 6-10 s (not faster than 1 turn per 2 s).");
  Serial.println("End exactly on your mark and hold still until STOP. The window is 30 s.");
  waitEnter();
  countdown();
  Win w;
  phase("GO - turn now, then hold still at the mark", 30000, w);
  Serial.println(">>> STOP");

  double deg = w.ang[2] * 180.0 / PI;
  Serial.printf("RESULT S2 gz integral = %.1f deg for one CCW turn (360 expected)\n", deg);
  Serial.printf("RESULT S2 gyro scale factor = %.3f (1.000 = perfect)\n", fabs(deg) / 360.0);
  Serial.printf("RESULT S2 gx,gy integrals = %.1f, %.1f deg (should be near 0)\n",
                w.ang[0] * 180.0 / PI, w.ang[1] * 180.0 / PI);
  if (deg > 0)
    Serial.println("RESULT S2 verdict: gz POSITIVE for CCW -> standard frame, REMOVE 'gz = -gz' in main.cpp");
  else
    Serial.println("RESULT S2 verdict: gz NEGATIVE for CCW -> keep the flip 'gz = -gz' (and tell me why: board orientation?)");
  if (fabs(deg) < 270 || fabs(deg) > 450)
    Serial.println("WARNING: far from 360 deg - not a full turn, or too fast. The sign is still valid, the scale is not. Re-run.");
}

// ---------------------------------------------------------------- stages 3 and 4
static void tiltStage(int n, const char *title, const char *how, const char *liftMsg, bool pitchAxis) {
  stageNo = n;
  Serial.printf("\n==== STAGE %d/4: %s ====\n", n, title);
  Serial.println(how);
  Serial.println("Timeline after GO: 3 s flat, 4 s lifting, 3 s holding the raised angle.");
  waitEnter();
  countdown();
  Win flat, lift, hold;
  phase("HOLD FLAT, hands steady", 3000, flat);
  phase(liftMsg, 4000, lift);
  phase("HOLD IT STILL at the raised angle", 3000, hold);
  Serial.println(">>> STOP - put it back down");

  double a0 = pitchAxis ? climbDeg(flat) : leftUpDeg(flat);
  double a1 = pitchAxis ? climbDeg(hold) : leftUpDeg(hold);
  double da = a1 - a0;
  // nose up = negative gy; left side up = positive gx
  double g = pitchAxis ? -(lift.ang[1] + hold.ang[1]) : (lift.ang[0] + hold.ang[0]);
  g *= 180.0 / PI;

  Serial.printf("RESULT S%d accel angle: flat=%.2f -> raised=%.2f deg (change %.2f deg)\n", n, a0, a1, da);
  Serial.printf("RESULT S%d gyro integral, same sign convention = %.2f deg\n", n, g);
  if (fabs(da) > 1.0) Serial.printf("RESULT S%d gyro/accel ratio = %.3f\n", n, g / da);
  Serial.printf("RESULT S%d raised-minus-flat accel: dax=%.2f day=%.2f daz=%.2f m/s2\n", n,
                mean(hold, 0) - mean(flat, 0), mean(hold, 1) - mean(flat, 1), mean(hold, 2) - mean(flat, 2));
  Serial.printf("RESULT S%d gyro std while holding: gx=%.4f gy=%.4f gz=%.4f rad/s\n", n,
                sdev(hold, 3), sdev(hold, 4), sdev(hold, 5));
  if (fabs(da) < 8.0)
    Serial.println("WARNING: tilt under 8 deg - too small for a good comparison. Re-run with a bigger lift.");
  else if ((da > 0) != (g > 0))
    Serial.println("RESULT: SIGN MISMATCH between accel and gyro - tell me, this matters.");
}

// ---------------------------------------------------------------- main
void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("\nIMU guided test - about 2 minutes, 4 stages");

  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom(0x68, 1, true) == 1)
    Serial.printf("RESULT WHO_AM_I=0x%02X (a genuine MPU6050 returns 0x68)\n", Wire.read());
  else
    Serial.println("RESULT WHO_AM_I read FAILED");

  auto rd = [](uint8_t reg) -> int {
  Wire.beginTransmission(0x68); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(0x68, 1, true) != 1) return -1;
  return Wire.read();
};
  auto wr = [](uint8_t reg, uint8_t val) {
    Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(val); Wire.endTransmission(true);
  };
  wr(0x6B, 0x01); delay(10);
  Serial.printf("RESULT REG GYRO_CONFIG(1B)=0x%02X ACCEL_CONFIG(1C)=0x%02X ACCEL_CONFIG2(1D)=0x%02X\n",
                rd(0x1B), rd(0x1C), rd(0x1D));
  wr(0x1D, 0x03);
  Serial.printf("RESULT REG 1D after write 0x03 = 0x%02X\n", rd(0x1D));
  wr(0x1D, 0x00);

  if (!imu.begin()) {
    Serial.println("RESULT imu.begin() FAILED - check wiring/address");
    while (true) delay(1000);
  }

  stage1();
  stage2();
  tiltStage(3, "NOSE UP",
            "Robot flat on the surface. You will lift the FRONT (nose) about 15-20 deg, e.g. rest it on a book.",
            "LIFT THE FRONT NOW, slowly", true);
  tiltStage(4, "LEFT SIDE UP",
            "Robot flat on the surface. You will lift the LEFT side (your left when standing behind it,\n"
            "looking the way it drives) about 15-20 deg.",
            "LIFT THE LEFT SIDE NOW, slowly", false);

  Serial.println("\n==== ALL DONE ====");
  Serial.println("Copy every line that starts with RESULT and paste it back. Press ENTER to run again.");
}

void loop() {
  if (Serial.available()) { delay(50); ESP.restart(); }
  delay(50);
}