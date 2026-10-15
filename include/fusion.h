// fusion.h - robust IMU + wheel-encoder fusion for a skid-steer robot (2D pose).
//
// State  x = [px, py, theta, v, b_g, b_a]
//   px,py,theta : pose in the odom frame        v   : forward speed [m/s]
//   b_g         : gyro-z bias [rad/s]           b_a : forward-accel bias [m/s^2]
// Inputs (prediction):  gz (yaw rate, driver-corrected), af (gravity-compensated forward accel)
// Measurements (sequential scalar updates, Joseph form, no matrix inverse):
//   1. encoder forward speed         z = v_enc          h = v        (R grows with slip / turning)
//   2. encoder yaw rate              z = w_enc          h = gz - b_g (informs the gyro bias only)
//   3. zero-velocity (when the wheels have really stopped):  v = 0,  gz - b_g = 0,  af - b_a = 0
// Robustness: innovation gating with lock-out protection, same-side wheel slip detection, turn-dependent
// encoder trust (skid-steer slip), actual-stillness detection from the encoders, encoder-only fallback when
// the IMU drops out, NaN/divergence guard.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

struct FusionParams {
  float track_width_eff = 0.20f;   // [m] effective (slip-corrected) track width for the encoder yaw rate
  // process noise: continuous spectral densities, multiplied by dt
  float q_xy = 1e-6f, q_theta = 2.5e-5f, q_v = 0.04f, q_bg = 1e-6f, q_ba = 1e-5f;
  float q_theta_enc_only = 1e-2f;  // extra heading noise per second while running without the IMU
  // encoder speed measurement
  float r_v = 4e-4f;               // (0.02 m/s)^2
  float r_v_turn = 0.01f;          // + variance per (rad/s)^2 of yaw rate
  float r_v_slip = 0.04f;          // variance when same-side wheels disagree
  float slip_thresh = 0.05f;       // [m/s] front/rear disagreement that counts as slip
  // encoder yaw-rate measurement (only informs b_g)
  float r_w = 2.5e-3f;             // (0.05 rad/s)^2
  float r_w_turn = 0.25f;          // + variance per (rad/s)^2: skid steer slips more the harder it turns
  // zero-velocity updates
  float still_time = 0.4f;         // [s] of zero encoder counts on all four wheels
  float r_zupt_v = 1e-6f, r_zupt_w = 1e-5f, r_zupt_a = 2.5e-3f;
  float zupt_w_max = 0.05f;        // [rad/s] only trust ZUPT if the gyro agrees the robot is not rotating
  // gating
  float gate = 3.0f;               // sigma
  int   max_rejects = 5;           // consecutive rejects before the measurement is accepted with inflated R
};

struct FusionInput {
  float dt;                        // [s]
  bool  imu_ok;                    // gz/af valid this cycle
  float gz;                        // [rad/s] FLU, initial gyro bias already removed
  float af;                        // [m/s^2] gravity-compensated forward acceleration
  float dfl, drl, dfr, drr;        // wheel travel this cycle [m], positive = robot forward
};

class FusionEKF {
public:
  FusionParams p;
  float x[6];
  float P[6][6];
  float omega = 0.0f;              // best yaw-rate estimate [rad/s]
  bool  still = false, slip_l = false, slip_r = false;
  uint32_t rejected_v = 0, rejected_w = 0, resets = 0;

  FusionEKF() { reset(); }

  void reset(float theta0 = 0.0f) {
    memset(x, 0, sizeof(x));
    memset(P, 0, sizeof(P));
    x[2] = theta0;
    P[0][0] = P[1][1] = 1e-4f;
    P[2][2] = 1e-2f; P[3][3] = 1e-2f; P[4][4] = 1e-4f; P[5][5] = 1e-2f;
    still_cycles_ = 0; rej_v_ = rej_w_ = 0;
    omega = 0.0f;
  }

  float px() const { return x[0]; }
  float py() const { return x[1]; }
  float theta() const { return x[2]; }
  float v() const { return x[3]; }
  float bg() const { return x[4]; }
  float ba() const { return x[5]; }

  // ---- one fusion cycle ----
  void step(const FusionInput& in) {
    const float dt = in.dt;
    if (!(dt > 0.0f) || dt > 0.5f) return;

    // wheel speeds, forward positive
    float vfl = in.dfl / dt, vrl = in.drl / dt, vfr = in.dfr / dt, vrr = in.drr / dt;
    float vl = side_speed(vfl, vrl, slip_l);
    float vr = side_speed(vfr, vrr, slip_r);
    float v_enc = 0.5f * (vl + vr);
    float w_enc = (vr - vl) / p.track_width_eff;

    // real stillness: no encoder tick on any wheel for still_time
    bool no_tick = (in.dfl == 0.0f && in.drl == 0.0f && in.dfr == 0.0f && in.drr == 0.0f);
    still_cycles_ = no_tick ? still_cycles_ + 1 : 0;
    still = (still_cycles_ * dt) >= p.still_time;

    // prediction
    if (in.imu_ok) {
      predict(dt, in.gz, in.af);
      omega = in.gz - x[4];
    } else {                                        // encoder-only fallback: heading from wheel yaw rate
      predict(dt, w_enc + x[4], x[5]);
      P[2][2] += p.q_theta_enc_only * dt;
      omega = w_enc;
    }

    // measurements
    float H[6];
    if (still) {
      zero(H); H[3] = 1.0f;
      update1(0.0f, x[3], H, p.r_zupt_v, nullptr);
      if (in.imu_ok && fabsf(in.gz - x[4]) < p.zupt_w_max) {
        zero(H); H[4] = -1.0f;
        update1(0.0f, in.gz - x[4], H, p.r_zupt_w, nullptr);          // gz = b_g
        zero(H); H[5] = -1.0f;
        update1(0.0f, in.af - x[5], H, p.r_zupt_a, nullptr);          // af = b_a
      }
      omega = in.imu_ok ? in.gz - x[4] : 0.0f;
    } else {
      float w2 = omega * omega;
      float Rv = p.r_v + p.r_v_turn * w2 + ((slip_l || slip_r) ? p.r_v_slip : 0.0f);
      zero(H); H[3] = 1.0f;
      if (!update1(v_enc, x[3], H, Rv, &rej_v_)) rejected_v++;
      if (in.imu_ok) {
        float Rw = p.r_w + p.r_w_turn * w2;
        zero(H); H[4] = -1.0f;
        if (!update1(w_enc, in.gz - x[4], H, Rw, &rej_w_)) rejected_w++;
        omega = in.gz - x[4];
      }
    }
    sanity();
  }

private:
  int still_cycles_ = 0, rej_v_ = 0, rej_w_ = 0;

  static void zero(float* H) { for (int i = 0; i < 6; i++) H[i] = 0.0f; }

  static float wrap(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
  }

  // Same-side front/rear wheels: average if they agree, otherwise take the slower one (a spinning wheel
  // over-reads) and flag slip so that the speed measurement gets a large variance.
  float side_speed(float vf, float vr, bool& slip) const {
    if (fabsf(vf - vr) > p.slip_thresh) {
      slip = true;
      return (fabsf(vf) < fabsf(vr)) ? vf : vr;
    }
    slip = false;
    return 0.5f * (vf + vr);
  }

  void predict(float dt, float gz, float af) {
    float w = gz - x[4];
    float a = af - x[5];
    float tm = x[2] + 0.5f * w * dt;
    float c = cosf(tm), s = sinf(tm);
    float v = x[3];

    float F[6][6];
    for (int i = 0; i < 6; i++) for (int j = 0; j < 6; j++) F[i][j] = (i == j) ? 1.0f : 0.0f;
    F[0][2] = -v * s * dt;  F[0][3] = c * dt;  F[0][4] =  0.5f * v * s * dt * dt;
    F[1][2] =  v * c * dt;  F[1][3] = s * dt;  F[1][4] = -0.5f * v * c * dt * dt;
    F[2][4] = -dt;
    F[3][5] = -dt;

    x[0] += v * c * dt;
    x[1] += v * s * dt;
    x[2]  = wrap(x[2] + w * dt);
    x[3] += a * dt;

    float FP[6][6], Pn[6][6];
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        float sum = 0;
        for (int k = 0; k < 6; k++) sum += F[i][k] * P[k][j];
        FP[i][j] = sum;
      }
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        float sum = 0;
        for (int k = 0; k < 6; k++) sum += FP[i][k] * F[j][k];     // (F P) F^T
        Pn[i][j] = sum;
      }
    Pn[0][0] += p.q_xy * dt;      Pn[1][1] += p.q_xy * dt;
    Pn[2][2] += p.q_theta * dt;   Pn[3][3] += p.q_v * dt;
    Pn[4][4] += p.q_bg * dt;      Pn[5][5] += p.q_ba * dt;
    memcpy(P, Pn, sizeof(P));
  }

  // Scalar measurement update (Joseph form). z: measurement, h: predicted measurement, H: row of the
  // Jacobian. rej: consecutive-reject counter for gating (nullptr = no gating). Returns false if rejected.
  bool update1(float z, float h, const float* H, float R, int* rej) {
    float PHt[6];
    for (int i = 0; i < 6; i++) {
      float sum = 0;
      for (int j = 0; j < 6; j++) sum += P[i][j] * H[j];
      PHt[i] = sum;
    }
    float HPHt = 0;
    for (int i = 0; i < 6; i++) HPHt += H[i] * PHt[i];
    float y = z - h;
    float S = HPHt + R;
    if (rej) {
      if (y * y > p.gate * p.gate * S) {
        if (++(*rej) < p.max_rejects) return false;
        R *= 25.0f;                        // persistent disagreement: accept, but with little weight
        S = HPHt + R;
      }
      *rej = 0;
    }
    if (!(S > 0.0f)) return false;

    float K[6];
    for (int i = 0; i < 6; i++) K[i] = PHt[i] / S;
    for (int i = 0; i < 6; i++) x[i] += K[i] * y;
    x[2] = wrap(x[2]);

    float A[6][6], AP[6][6], Pn[6][6];
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) A[i][j] = ((i == j) ? 1.0f : 0.0f) - K[i] * H[j];
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        float sum = 0;
        for (int k = 0; k < 6; k++) sum += A[i][k] * P[k][j];
        AP[i][j] = sum;
      }
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) {
        float sum = 0;
        for (int k = 0; k < 6; k++) sum += AP[i][k] * A[j][k];     // A P A^T
        Pn[i][j] = sum + R * K[i] * K[j];
      }
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++) P[i][j] = 0.5f * (Pn[i][j] + Pn[j][i]);   // keep symmetric
    return true;
  }

  void sanity() {
    bool ok = true;
    for (int i = 0; i < 6; i++) {
      if (!isfinite(x[i]) || !isfinite(P[i][i]) || P[i][i] < 0.0f) ok = false;
      if (P[i][i] > 1e6f) P[i][i] = 1e6f;
    }
    if (fabsf(x[3]) > 5.0f || fabsf(x[4]) > 1.0f || fabsf(x[5]) > 5.0f) ok = false;   // physically absurd
    if (!ok) {
      float th = isfinite(x[2]) ? x[2] : 0.0f, px = isfinite(x[0]) ? x[0] : 0.0f, py = isfinite(x[1]) ? x[1] : 0.0f;
      reset(th);
      x[0] = px; x[1] = py;
      resets++;
    }
  }
};
