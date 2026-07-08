#include "ImuProcessor.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "config.h"

namespace {
float dot3(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}
float norm3(const float a[3]) { return sqrtf(dot3(a, a)); }
void normalize3(float a[3]) {
  float n = norm3(a);
  if (n > 1e-6f) {
    a[0] /= n; a[1] /= n; a[2] /= n;
  }
}
}  // namespace

bool ImuProcessor::begin() {
  ok_ = sensor_.begin(Wire);
  if (!ok_) {
    log_e("Supported motion sensor not found (MPU6050/MPU6500 at 0x68/0x69)");
    return false;
  }
  log_i("Motion sensor: %s at 0x%02X (WHO_AM_I=0x%02X)", sensor_.name(),
        sensor_.address(), sensor_.identity());

  loadCalibration();
  last_sample_us_ = micros();
  return true;
}

// --- Persistence -----------------------------------------------------------
void ImuProcessor::loadCalibration() {
  if (!imu::PERSIST_CALIBRATION) return;
  prefs_.begin("imu", true /* read-only */);
  uint32_t ver = prefs_.getUInt("ver", 0);
  if (ver == imu::CAL_VERSION && prefs_.isKey("fwd0")) {
    fwd_[0] = prefs_.getFloat("fwd0", 1);
    fwd_[1] = prefs_.getFloat("fwd1", 0);
    fwd_[2] = prefs_.getFloat("fwd2", 0);
    up_[0]  = prefs_.getFloat("up0", 0);
    up_[1]  = prefs_.getFloat("up1", 0);
    up_[2]  = prefs_.getFloat("up2", 1);
    cal_weight_ = prefs_.getFloat("wt", 0);
    for (int i = 0; i < 3; ++i) gravity_[i] = up_[i];
    normalize3(up_);
    normalize3(fwd_);
    cross3(fwd_, up_, lat_);  // right-pointing, must match updateOrientation
    normalize3(lat_);
    log_i("Loaded IMU calibration (weight=%.1f)", cal_weight_);
  }
  prefs_.end();
}

void ImuProcessor::saveCalibration() {
  if (!imu::PERSIST_CALIBRATION) return;
  prefs_.begin("imu", false);
  prefs_.putUInt("ver", imu::CAL_VERSION);
  prefs_.putFloat("fwd0", fwd_[0]);
  prefs_.putFloat("fwd1", fwd_[1]);
  prefs_.putFloat("fwd2", fwd_[2]);
  prefs_.putFloat("up0", up_[0]);
  prefs_.putFloat("up1", up_[1]);
  prefs_.putFloat("up2", up_[2]);
  prefs_.putFloat("wt", cal_weight_);
  prefs_.end();
  cal_dirty_ = false;
}

void ImuProcessor::resetCalibration() {
  cal_weight_ = 0;
  peak_pos_ = peak_neg_ = 0;
  primed_ = false;
  if (imu::PERSIST_CALIBRATION) {
    prefs_.begin("imu", false);
    prefs_.clear();
    prefs_.end();
  }
}

// --- Main sampling ---------------------------------------------------------
ImuEvent ImuProcessor::update() {
  drive_just_ended_ = false;
  if (!ok_) return ImuEvent::None;

  const uint32_t period_us = 1000000UL / imu::SAMPLE_HZ;
  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_sample_us_) < period_us) return ImuEvent::None;

  float dt = (now_us - last_sample_us_) / 1e6f;
  last_sample_us_ = now_us;
  if (dt <= 0 || dt > 0.5f) dt = 1.0f / imu::SAMPLE_HZ;

  sample(dt);
  return classify();
}

void ImuProcessor::sample(float dt) {
  MotionSample reading;
  if (!sensor_.read(reading)) {
    log_w("Motion sensor burst read failed");
    return;
  }
  float acc[3] = {reading.accel_g[0], reading.accel_g[1], reading.accel_g[2]};
  float gyro[3] = {reading.gyro_dps[0], reading.gyro_dps[1],
                   reading.gyro_dps[2]};

  // --- Gyro zero-rate bias ---
  // Learn the standing offset whenever the device is quiet (uses the previous
  // sample's dynamic values — one frame of lag is irrelevant at 20 s tau), and
  // always subtract it. Without this, a typical few-°/s bias defeats both the
  // corner gate and the "driving straight" calibration gate.
  bool quiet = !driving_ &&
               fabsf(fwd_g_) < imu::QUIET_ACCEL_G &&
               fabsf(lat_g_) < imu::QUIET_ACCEL_G &&
               fabsf(vert_g_) < imu::QUIET_ACCEL_G;
  float ba = dt / (imu::GYRO_BIAS_TAU_S + dt);
  for (int i = 0; i < 3; ++i) {
    if (quiet && fabsf(gyro[i] - gyro_bias_[i]) < imu::QUIET_GYRO_DPS) {
      gyro_bias_[i] += (gyro[i] - gyro_bias_[i]) * ba;
    }
    gyro[i] -= gyro_bias_[i];
  }

  updateOrientation(acc, gyro, dt);

  // --- Per-drive stat accumulation ---
  if (driving_) {
    float braking = fwd_g_ < 0 ? -fwd_g_ : 0.0f;
    float accel   = fwd_g_ > 0 ? fwd_g_ : 0.0f;
    float corner  = fabsf(lat_g_);
    float turn    = fabsf(yaw_dps_);
    if (braking > stats_.max_brake_g)  stats_.max_brake_g  = braking;
    if (accel   > stats_.max_accel_g)  stats_.max_accel_g  = accel;
    if (corner  > stats_.max_corner_g) stats_.max_corner_g = corner;
    if (turn    > stats_.max_turn_dps) stats_.max_turn_dps = turn;

    jerk_accum_ += fabsf(fwd_g_ - last_fwd_g_);
    jerk_samples_++;
  }
  last_fwd_g_ = fwd_g_;

  // "Moving" = horizontal g OR road vibration OR real turning. Horizontal g
  // alone fails on smooth highway cruising, which would end the drive early.
  bool moving = sqrtf(fwd_g_ * fwd_g_ + lat_g_ * lat_g_) > imu::MOTION_G ||
                fabsf(vert_g_) > imu::MOTION_VERT_G ||
                fabsf(yaw_dps_) > imu::MOTION_YAW_DPS;
  updateSession(moving, millis());
}

// Estimate the car-frame axes and project the current sample onto them.
void ImuProcessor::updateOrientation(const float acc[3], const float gyro[3],
                                     float dt) {
  if (!primed_) {
    for (int i = 0; i < 3; ++i) gravity_[i] = acc[i];
    primed_ = true;
    if (cal_weight_ < 1e-3f) seedForward();
  }

  // Gravity via slow low-pass → vertical axis. This tracks the mount angle and
  // any slow tilt without integrating drift. Freeze the filter while the accel
  // VECTOR deviates from the gravity estimate — otherwise a 3-5 s braking or
  // cornering event leaks in, tilting "up" and decaying the reading mid-event.
  // (The vector deviation is first-order in the maneuver strength; total
  // magnitude |acc|-1 is only second-order and misses moderate braking.)
  float dev[3] = {acc[0] - gravity_[0], acc[1] - gravity_[1],
                  acc[2] - gravity_[2]};
  float ga = dt / (imu::GRAVITY_TAU_S + dt);
  if (norm3(dev) > imu::GRAVITY_FREEZE_DEV_G) ga = 0.0f;
  for (int i = 0; i < 3; ++i)
    gravity_[i] += (acc[i] - gravity_[i]) * ga;
  for (int i = 0; i < 3; ++i) up_[i] = gravity_[i];
  normalize3(up_);

  // Dynamic acceleration = raw minus gravity.
  float dyn[3] = {acc[0] - gravity_[0], acc[1] - gravity_[1],
                  acc[2] - gravity_[2]};

  // Split into vertical + horizontal components.
  vert_g_ = dot3(dyn, up_);
  float horiz[3] = {dyn[0] - vert_g_ * up_[0], dyn[1] - vert_g_ * up_[1],
                    dyn[2] - vert_g_ * up_[2]};

  // Turn-rate about the vertical axis. Gyro follows the right-hand rule, so
  // +rotation about "up" is a LEFT turn; negate so positive = RIGHT turn,
  // matching the lateral axis below.
  yaw_dps_ = -dot3(gyro, up_);

  // Learn the forward axis from straight-line longitudinal accel.
  updateCalibration(horiz);

  // Keep forward strictly horizontal and rebuild the lateral axis.
  // lat = fwd x up points RIGHT (fwd=x, up=z: x cross z = -y = right), so a
  // right turn's centripetal accel gives lat_g_ > 0. Both turning signals
  // agree: positive = right.
  float f_dot_up = dot3(fwd_, up_);
  float fwd_h[3] = {fwd_[0] - f_dot_up * up_[0], fwd_[1] - f_dot_up * up_[1],
                    fwd_[2] - f_dot_up * up_[2]};
  normalize3(fwd_h);
  for (int i = 0; i < 3; ++i) fwd_[i] = fwd_h[i];
  cross3(fwd_, up_, lat_);
  normalize3(lat_);

  // Project into car frame. A negative FORWARD_SIGN_OVERRIDE flips the
  // reported fore-aft (and, to keep handedness, left-right) without disturbing
  // the learned axis, so it can't fight the calibrator.
  float signMul = imu::FORWARD_SIGN_OVERRIDE < 0 ? -1.0f : 1.0f;
  fwd_g_ = dot3(horiz, fwd_) * signMul;
  lat_g_ = dot3(horiz, lat_) * signMul * imu::LATERAL_SIGN;
}

// Pick an initial forward guess: the chip axis least aligned with gravity,
// projected into the horizontal plane. Good enough until learning refines it.
void ImuProcessor::seedForward() {
  float best[3] = {1, 0, 0};
  float bestDot = 2.0f;
  const float axes[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  float u[3] = {gravity_[0], gravity_[1], gravity_[2]};
  normalize3(u);
  for (int k = 0; k < 3; ++k) {
    float d = fabsf(dot3(axes[k], u));
    if (d < bestDot) { bestDot = d; best[0] = axes[k][0]; best[1] = axes[k][1]; best[2] = axes[k][2]; }
  }
  float d = dot3(best, u);
  for (int i = 0; i < 3; ++i) fwd_[i] = best[i] - d * u[i];
  normalize3(fwd_);
}

void ImuProcessor::updateCalibration(const float horiz[3]) {
  float mag = norm3(horiz);
  bool straight = fabsf(yaw_dps_) < imu::CAL_STRAIGHT_YAW_DPS;
  if (!straight || mag < imu::CAL_MIN_ACCEL_G) return;

  // Observed longitudinal direction this sample.
  float obs[3] = {horiz[0] / mag, horiz[1] / mag, horiz[2] / mag};

  // The axis is a line, not a ray — flip the observation into the same
  // hemisphere as the current estimate before averaging.
  if (dot3(obs, fwd_) < 0)
    for (int i = 0; i < 3; ++i) obs[i] = -obs[i];

  // EMA the forward axis toward the observation, weighted by how strong the
  // accel was (stronger events are more trustworthy).
  float w = imu::CAL_LEARN_RATE * fminf(mag / imu::CAL_MIN_ACCEL_G, 3.0f);
  for (int i = 0; i < 3; ++i) fwd_[i] += (obs[i] - fwd_[i]) * w;
  normalize3(fwd_);

  cal_weight_ += imu::CAL_LEARN_RATE;
  cal_dirty_ = true;

  // Forward-sign detection (only when not overridden): braking pushes the
  // reading toward the rear and is usually harder than accelerating, so the
  // stronger-peak direction is the rear. Track peaks along the axis.
  float proj = dot3(horiz, fwd_);
  if (proj > 0) peak_pos_ = fmaxf(peak_pos_, proj);
  else          peak_neg_ = fmaxf(peak_neg_, -proj);

  if (imu::FORWARD_SIGN_OVERRIDE == 0 && peak_pos_ > 0.05f &&
      peak_neg_ > 0.05f && peak_pos_ > peak_neg_) {
    // +fwd_ is the harder (braking → rear) direction: flip so + = front.
    for (int i = 0; i < 3; ++i) fwd_[i] = -fwd_[i];
    float tmp = peak_pos_; peak_pos_ = peak_neg_; peak_neg_ = tmp;
  }

  // Persist occasionally once confident, not every sample (flash wear).
  if (imu::PERSIST_CALIBRATION && calibrated()) {
    uint32_t now = millis();
    if (now - last_save_ms_ > 60000) {
      last_save_ms_ = now;
      saveCalibration();
    }
  }
}

void ImuProcessor::updateSession(bool moving, uint32_t now) {
  if (moving) {
    still_since_ms_ = 0;
    if (motion_since_ms_ == 0) motion_since_ms_ = now;
    if (!driving_ && (now - motion_since_ms_) >= imu::DRIVE_START_MS) {
      driving_ = true;
      stats_ = DriveStats{};
      stats_.start_ms = now;
      jerk_accum_ = 0;
      jerk_samples_ = 0;
      log_i("Drive started");
    }
  } else {
    motion_since_ms_ = 0;
    if (still_since_ms_ == 0) still_since_ms_ = now;
    if (driving_ && (now - still_since_ms_) >= imu::DRIVE_END_MS) {
      stats_.duration_s = (still_since_ms_ - stats_.start_ms) / 1000;

      float mean_jerk =
          jerk_samples_ ? (float)(jerk_accum_ / jerk_samples_) : 0.0f;
      float penalty = (stats_.hard_brake_count + stats_.hard_corner_count) * 4.0f;
      penalty += mean_jerk * 400.0f;
      float score = 100.0f - penalty;
      stats_.smoothness = (uint8_t)lroundf(constrain(score, 0.0f, 100.0f));

      driving_ = false;
      drive_just_ended_ = true;
      if (cal_dirty_) saveCalibration();
      log_i("Drive ended: %us brakes=%u corners=%u turnpk=%.0f smooth=%u",
            stats_.duration_s, stats_.hard_brake_count,
            stats_.hard_corner_count, stats_.max_turn_dps, stats_.smoothness);
    }
  }
}

ImuEvent ImuProcessor::classify() {
  const uint32_t now = millis();

  float braking = fwd_g_ < 0 ? -fwd_g_ : 0.0f;
  float accel   = fwd_g_ > 0 ? fwd_g_ : 0.0f;
  float corner  = fabsf(lat_g_);
  float bump    = fabsf(vert_g_);
  bool  turning = fabsf(yaw_dps_) >= imu::CORNER_YAW_GATE_DPS;

  // Note: events still fire when parked (fun for demos — shake it, it reacts)
  // but the hard-event COUNTERS only accumulate during a drive, so post-drive
  // handling can't pollute a summary that's about to be logged.
  if (braking >= imu::BRAKE_G && (now - cd_brake_) >= imu::EVENT_COOLDOWN_MS) {
    cd_brake_ = now;
    if (driving_ && braking >= imu::HARD_BRAKE_G) stats_.hard_brake_count++;
    return ImuEvent::HardBrake;
  }
  if (bump >= imu::BUMP_G && (now - cd_bump_) >= imu::EVENT_COOLDOWN_MS) {
    cd_bump_ = now;
    return ImuEvent::Bump;
  }
  // A corner requires BOTH lateral g AND real rotation — a side-bump alone
  // won't trigger it. Direction comes from the gyro (a direct rotation
  // measurement) rather than the bump-contaminated accelerometer.
  if (corner >= imu::CORNER_G && turning &&
      (now - cd_corner_) >= imu::EVENT_COOLDOWN_MS) {
    cd_corner_ = now;
    if (driving_ && corner >= imu::HARD_CORNER_G) stats_.hard_corner_count++;
    return yaw_dps_ > 0 ? ImuEvent::CornerRight : ImuEvent::CornerLeft;
  }
  if (accel >= imu::ACCEL_G && (now - cd_accel_) >= imu::EVENT_COOLDOWN_MS) {
    cd_accel_ = now;
    return ImuEvent::Accelerate;
  }
  return ImuEvent::None;
}
