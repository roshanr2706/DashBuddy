// ============================================================================
//  FaceEngine.h — layered emotion controller + Grobot-inspired vector renderer.
//
//  FaceEngine still owns the three behavioural layers:
//    1. persistent driving mood/tension
//    2. short, intensity-aware event reactions
//    3. idle blinks and glances
//
//  The visual state is a set of spring-animated geometric parameters. This
//  gives the rounded, pupil-free robot eyes and subtle overshoot associated
//  with expressive OLED faces without coupling driving logic to a library.
// ============================================================================
#pragma once

#include <stdint.h>

#include "ImuProcessor.h"

class TFT_eSprite;

struct FaceEventContext {
  float forward_g = 0.0f;
  float lateral_g = 0.0f;
  float vertical_g = 0.0f;
  float turn_rate_dps = 0.0f;
};

class FaceEngine {
 public:
  void begin();
  void onEvent(ImuEvent event, const FaceEventContext& motion = {});
  void update(uint32_t now, float forwardG, float lateralG, bool driving);
  void render(TFT_eSprite& canvas);

 private:
  struct VisualState {
    float eye_width = 38.0f;
    float eye_height = 43.0f;
    float eye_radius = 9.0f;
    float eye_asymmetry = 0.0f;
    float top_lid = 1.0f;
    float bottom_lid = 1.0f;
    float lid_tilt = 0.0f;
    float face_x = 0.0f;
    float face_y = 0.0f;
    float rotation = 0.0f;
  } current_, target_, velocity_;

  enum class Reaction : uint8_t {
    None, Brake, Accelerate, Corner, Bump, Dizzy, Dead
  };

  void spring(float& value, float& velocity, float target, float dt,
              float stiffness, float damping);
  void setMoodTargets(float lateralG);
  void applyReactionTargets(uint32_t now);
  uint16_t moodBackground() const;
  uint16_t moodForeground() const;

  float tension_ = 0.0f;
  float flash_ = 0.0f;
  float jiggle_ = 0.0f;
  float event_intensity_ = 0.0f;
  float turn_direction_ = 0.0f;

  Reaction reaction_ = Reaction::None;
  uint32_t reaction_until_ = 0;
  uint32_t spin_started_ = 0;
  uint32_t dizzy_until_ = 0;
  uint32_t dead_until_ = 0;
  uint32_t next_blink_ = 0;
  uint32_t blink_until_ = 0;
  uint32_t next_glance_ = 0;
  uint32_t glance_until_ = 0;
  float glance_direction_ = 0.0f;
  uint32_t last_ms_ = 0;
  float animation_time_ = 0.0f;
};
