#pragma once
#include <math.h>

class Attitude {
public:
  static constexpr float G = 9.80665f;
  float roll = 0, pitch = 0;   // rad; pitch+ = nose down, roll+ = left side up
  bool  ramp = false;

  void calibrate(const float a[3], const float g[3]) {
    k_ = G / sqrtf(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    gb_[0] = g[0]; gb_[1] = g[1]; gb_[2] = 0;   // z handled by driver + ZUPT
    float ax = a[0]*k_, ay = a[1]*k_, az = a[2]*k_;
    pitch0_ = atan2f(-ax, hypotf(ay, az));
    roll0_  = atan2f(ay, az);
    roll = pitch = 0; ramp = false; t_ = 0;
  }

  void update(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    ax *= k_; ay *= k_; az *= k_;
    ax_ = ax;
    float p = gx - gb_[0], q = gy - gb_[1], r = gz;

    float sr = sinf(roll), cr = cosf(roll), tp = tanf(pitch);
    float rollG  = roll  + (p + (q*sr + r*cr)*tp) * dt;
    float pitchG = pitch + (q*cr - r*sr) * dt;

    float n = sqrtf(ax*ax + ay*ay + az*az);
    float alpha = (fabsf(n - G) < 0.15f*G) ? TAU/(TAU + dt) : 1.0f;
    float rollA  = atan2f(ay, az) - roll0_;
    float pitchA = atan2f(-ax, hypotf(ay, az)) - pitch0_;
    roll  = alpha*rollG  + (1.0f - alpha)*rollA;
    pitch = alpha*pitchG + (1.0f - alpha)*pitchA;

    float ap = fabsf(pitch);
    if (!ramp) { t_ = (ap > RAMP_ON)  ? t_ + dt : 0; if (t_ > 0.3f) { ramp = true;  t_ = 0; } }
    else       { t_ = (ap < RAMP_OFF) ? t_ + dt : 0; if (t_ > 0.3f) { ramp = false; t_ = 0; } }
  }

  float forwardAccel() const { return ax_ + G * sinf(pitch); }
  float gainK() const { return k_; }
  float gyroBias(int i) const { return gb_[i]; }

private:
  static constexpr float TAU = 1.0f;
  static constexpr float RAMP_ON  = 0.1047f;   // 6 deg
  static constexpr float RAMP_OFF = 0.0698f;   // 4 deg
  float k_ = 1.0f, gb_[3] = {0, 0, 0}, pitch0_ = 0, roll0_ = 0, ax_ = 0, t_ = 0;
};