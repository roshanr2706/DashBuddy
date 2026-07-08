// ============================================================================
//  ImuProcessor.h — MPU6050/MPU6500 sampling, orientation auto-detection, and event
//  detection with first-class turning (§7).
//
//  Orientation is figured out at runtime, so the board can be mounted at any
//  angle (§7: "auto-detect the dominant horizontal axis while driving"):
//    • Up      — direction of gravity (slow low-pass of the accelerometer).
//    • Forward — learned from straight-line braking/accelerating (yaw ~ 0).
//    • Lateral — perpendicular in the horizontal plane; this is turning g.
//  The learned frame is persisted to NVS so it calibrates once and survives
//  reboots.
//
//  Turning is measured two ways: the gyro gives true turn-rate (°/s about the
//  vertical axis), and the lateral acceleration gives cornering g. A corner is
//  only counted when both agree, so a side-bump can't fake it.
// ============================================================================
#pragma once

#include <Preferences.h>
#include <stdint.h>

#include "config.h"  // inline accessors below use imu:: constants
#include "MotionSensor.h"

enum class ImuEvent : uint8_t {
  None,
  HardBrake,
  Accelerate,
  CornerLeft,
  CornerRight,
  Bump,
};

// Rolling statistics for one drive. Reset at drive start, snapshotted at end.
struct DriveStats {
  float    max_brake_g   = 0.0f;
  float    max_corner_g  = 0.0f;   // peak turning (lateral) g
  float    max_accel_g   = 0.0f;
  float    max_turn_dps  = 0.0f;   // peak turn-rate (gyro), °/s
  uint16_t hard_brake_count  = 0;
  uint16_t hard_corner_count = 0;
  uint32_t start_ms      = 0;
  uint32_t duration_s    = 0;
  uint8_t  smoothness    = 100;
};

class ImuProcessor {
 public:
  bool begin();

  // Call every loop; rate-limits to imu::SAMPLE_HZ internally.
  ImuEvent update();

  // Live signed values (g / °/s), gravity removed, in car-frame axes.
  // Sign convention: forward + = speeding up; lateral/turn + = right turn
  // (both turning signals share the sign, so they can be cross-checked).
  float forwardG()   const { return fwd_g_; }
  float lateralG()   const { return lat_g_; }
  float verticalG()  const { return vert_g_; }
  float turningG()   const { return lat_g_; }    // clearer alias
  float turnRateDps() const { return yaw_dps_; }

  // --- Orientation calibration ---
  bool  calibrated() const { return cal_weight_ >= imu::CAL_CONFIDENT_WEIGHT; }
  float calibrationProgress() const {  // 0..1
    float p = cal_weight_ / imu::CAL_CONFIDENT_WEIGHT;
    return p > 1.0f ? 1.0f : p;
  }
  void  resetCalibration();

  // --- Drive session state ---
  bool driving() const { return driving_; }
  const DriveStats& stats() const { return stats_; }
  bool driveJustEnded() const { return drive_just_ended_; }

 private:
  void     sample(float dt);
  void     updateOrientation(const float acc[3], const float gyro[3], float dt);
  void     updateCalibration(const float horiz[3]);
  void     seedForward();
  void     updateSession(bool moving, uint32_t now);
  ImuEvent classify();
  void     loadCalibration();
  void     saveCalibration();

  MotionSensor sensor_;
  Preferences prefs_;
  bool ok_ = false;

  uint32_t last_sample_us_ = 0;

  // --- Auto-detected orientation frame (unit vectors, chip coords) ---
  float gravity_[3] = {0, 0, 1};   // low-passed raw accel (g); |g| ~ 1
  float up_[3]      = {0, 0, 1};   // normalized gravity
  float fwd_[3]     = {1, 0, 0};   // forward unit (horizontal)
  float lat_[3]     = {0, -1, 0};  // lateral unit = fwd x up (points right)
  float gyro_bias_[3] = {0, 0, 0}; // learned zero-rate offset (°/s)
  bool  primed_     = false;
  float cal_weight_ = 0.0f;        // accumulated calibration evidence
  bool  cal_dirty_  = false;       // needs a save
  uint32_t last_save_ms_ = 0;

  // Forward-sign detection: peak |longitudinal accel| seen in each direction.
  float peak_pos_ = 0.0f, peak_neg_ = 0.0f;

  // --- Latest car-frame values ---
  float fwd_g_ = 0, lat_g_ = 0, vert_g_ = 0, yaw_dps_ = 0;

  // Event cooldown timestamps (ms) per kind.
  uint32_t cd_brake_ = 0, cd_accel_ = 0, cd_corner_ = 0, cd_bump_ = 0;

  // Drive session.
  bool     driving_ = false;
  bool     drive_just_ended_ = false;
  uint32_t motion_since_ms_ = 0;
  uint32_t still_since_ms_  = 0;
  DriveStats stats_;

  // Smoothness accumulation.
  double   jerk_accum_ = 0.0;
  uint32_t jerk_samples_ = 0;
  float    last_fwd_g_ = 0.0f;
};
