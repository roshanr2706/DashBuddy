#include "FaceEngine.h"

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <math.h>

#include "config.h"

namespace {
constexpr float kPi = 3.14159265359f;

uint32_t randomRange(uint32_t lo, uint32_t hi) {
  return hi <= lo ? lo : lo + esp_random() % (hi - lo);
}

float clamp01(float value) { return constrain(value, 0.0f, 1.0f); }

uint16_t blend565(uint16_t a, uint16_t b, float amount) {
  amount = clamp01(amount);
  int ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
  int br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
  int r = ar + static_cast<int>((br - ar) * amount);
  int g = ag + static_cast<int>((bg - ag) * amount);
  int bl = ab + static_cast<int>((bb - ab) * amount);
  return (r << 11) | (g << 5) | bl;
}

struct Point {
  int16_t x;
  int16_t y;
};

Point transformPoint(float x, float y, float cx, float cy, float degrees) {
  float radians = degrees * kPi / 180.0f;
  float cs = cosf(radians), sn = sinf(radians);
  return {static_cast<int16_t>(lroundf(cx + x * cs - y * sn)),
          static_cast<int16_t>(lroundf(cy + x * sn + y * cs))};
}

void fillQuad(TFT_eSprite& c, Point a, Point b, Point d, Point e,
              uint16_t color) {
  c.fillTriangle(a.x, a.y, b.x, b.y, d.x, d.y, color);
  c.fillTriangle(a.x, a.y, d.x, d.y, e.x, e.y, color);
}

void fillRotatedRect(TFT_eSprite& c, float x, float y, float w, float h,
                     float cx, float cy, float rotation, uint16_t color) {
  Point a = transformPoint(x, y, cx, cy, rotation);
  Point b = transformPoint(x + w, y, cx, cy, rotation);
  Point d = transformPoint(x + w, y + h, cx, cy, rotation);
  Point e = transformPoint(x, y + h, cx, cy, rotation);
  fillQuad(c, a, b, d, e, color);
}

void fillRoundedRotatedRect(TFT_eSprite& c, float x, float y, float w, float h,
                            float radius, float cx, float cy, float rotation,
                            uint16_t color) {
  radius = constrain(radius, 1.0f, fminf(w, h) * 0.5f);
  fillRotatedRect(c, x + radius, y, w - 2.0f * radius, h,
                  cx, cy, rotation, color);
  fillRotatedRect(c, x, y + radius, w, h - 2.0f * radius,
                  cx, cy, rotation, color);
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      Point corner = transformPoint(x + (sx > 0 ? w - radius : radius),
                                    y + (sy > 0 ? h - radius : radius),
                                    cx, cy, rotation);
      c.fillSmoothCircle(corner.x, corner.y, static_cast<int>(radius), color);
    }
  }
}
}  // namespace

void FaceEngine::begin() {
  last_ms_ = millis();
  next_blink_ = last_ms_ + randomRange(face_anim::BLINK_MIN_MS,
                                        face_anim::BLINK_MAX_MS);
  next_glance_ = last_ms_ + randomRange(face_anim::GLANCE_MIN_MS,
                                         face_anim::GLANCE_MAX_MS);
}

void FaceEngine::spring(float& value, float& velocity, float target, float dt,
                        float stiffness, float damping) {
  velocity += ((target - value) * stiffness - velocity * damping) * dt;
  value += velocity * dt;
}

void FaceEngine::onEvent(ImuEvent event, const FaceEventContext& motion) {
  const uint32_t now = millis();
  if (now < dead_until_) return;
  switch (event) {
    case ImuEvent::HardBrake:
      reaction_ = Reaction::Brake;
      event_intensity_ = clamp01((-motion.forward_g - imu::BRAKE_G) /
                                 (imu::HARD_BRAKE_G - imu::BRAKE_G + 0.01f));
      flash_ = 1.0f;
      reaction_until_ = now + face_anim::BRAKE_DURATION_MS;
      tension_ = fminf(1.0f, tension_ + 0.22f + 0.12f * event_intensity_);
      break;

    case ImuEvent::Accelerate:
      reaction_ = Reaction::Accelerate;
      event_intensity_ = clamp01(motion.forward_g / (imu::ACCEL_G * 1.8f));
      reaction_until_ = now + face_anim::ACCEL_DURATION_MS;
      break;

    case ImuEvent::CornerLeft:
    case ImuEvent::CornerRight: {
      reaction_ = Reaction::Corner;
      turn_direction_ = event == ImuEvent::CornerRight ? 1.0f : -1.0f;
      event_intensity_ = clamp01((fabsf(motion.lateral_g) - imu::CORNER_G) /
                                 (face_anim::SPIN_TRIGGER_G - imu::CORNER_G));
      reaction_until_ = now + face_anim::CORNER_DURATION_MS;
      tension_ = fminf(1.0f, tension_ + 0.08f + 0.08f * event_intensity_);

      bool hardEnoughToSpin =
          fabsf(motion.lateral_g) >= face_anim::SPIN_TRIGGER_G ||
          fabsf(motion.turn_rate_dps) >= face_anim::SPIN_TRIGGER_DPS;
      if (hardEnoughToSpin) {
        spin_started_ = now;
        reaction_until_ = now + face_anim::SPIN_DURATION_MS;
      }
      break;
    }

    case ImuEvent::Bump:
      reaction_ = Reaction::Bump;
      event_intensity_ = clamp01(fabsf(motion.vertical_g) / (imu::BUMP_G * 1.8f));
      jiggle_ = face_anim::BUMP_JIGGLE_PX * (0.65f + 0.55f * event_intensity_);
      reaction_until_ = now + face_anim::BUMP_DURATION_MS;
      break;

    case ImuEvent::None:
    default:
      break;
  }
}

void FaceEngine::setMoodTargets(float lateralG) {
  // Layer 1: continuous calm -> tense expression, plus live corner lean.
  target_.eye_width = 38.0f - 3.0f * tension_;
  target_.eye_height = 43.0f - 8.0f * tension_;
  target_.eye_radius = 9.0f;
  target_.eye_asymmetry = 0.0f;
  target_.top_lid = 1.0f + 5.0f * tension_;
  target_.bottom_lid = 1.0f;
  target_.lid_tilt = 10.0f * tension_;
  target_.face_x = -constrain(lateralG * face_anim::LIVE_LEAN_PX_PER_G,
                              -face_anim::MAX_LIVE_LEAN_PX,
                              face_anim::MAX_LIVE_LEAN_PX);
  target_.face_y = 0.0f;
  target_.rotation = -constrain(lateralG / imu::CORNER_G, -1.0f, 1.0f) *
                     face_anim::TURN_ROTATION_DEG;
}

void FaceEngine::applyReactionTargets(uint32_t now) {
  if (reaction_ == Reaction::Dead && now < dead_until_) {
    target_.eye_width = 38.0f;
    target_.eye_height = 42.0f;
    target_.eye_radius = 7.0f;
    target_.eye_asymmetry = 0.0f;
    target_.top_lid = 0.0f;
    target_.bottom_lid = 0.0f;
    target_.lid_tilt = 0.0f;
    target_.face_x = 0.0f;
    target_.face_y = 0.0f;
    target_.rotation = 0.0f;
    return;
  }

  if (spin_started_ != 0) {
    float phase = clamp01((now - spin_started_) /
                          static_cast<float>(face_anim::SPIN_DURATION_MS));
    // Smooth-step the single revolution so both ends settle cleanly.
    float eased = phase * phase * (3.0f - 2.0f * phase);
    target_.rotation = turn_direction_ * 360.0f * eased;
    // The revolution itself follows an explicit smooth-step path; the spring
    // owns entry lean and post-spin recovery. This guarantees one complete
    // rotation instead of letting spring lag shorten the turn.
    current_.rotation = target_.rotation;
    velocity_.rotation = 0.0f;
    target_.face_x = turn_direction_ * sinf(phase * kPi) * 10.0f;
    target_.eye_width = 35.0f;
    target_.eye_height = 34.0f;
    if (phase >= 1.0f) {
      spin_started_ = 0;
      reaction_ = Reaction::Dizzy;
      dizzy_until_ = now + face_anim::DIZZY_DURATION_MS;
      reaction_until_ = dizzy_until_;
      // Rotation is periodic; reset to the equivalent resting angle so the
      // spring does not unwind a full revolution after the explicit spin.
      current_.rotation -= turn_direction_ * 360.0f;
      target_.rotation = 0.0f;
    }
    return;
  }

  if (reaction_ == Reaction::Dizzy && now < dizzy_until_) {
    float remaining = (dizzy_until_ - now) /
                      static_cast<float>(face_anim::DIZZY_DURATION_MS);
    target_.rotation = sinf(animation_time_ * 8.0f) *
                       face_anim::DIZZY_WOBBLE_DEG * remaining;
    target_.face_x += sinf(animation_time_ * 5.0f) * 3.0f * remaining;
    target_.eye_width = 34.0f;
    target_.eye_height = 31.0f;
    target_.eye_asymmetry = sinf(animation_time_ * 7.0f) * 10.0f * remaining;
    target_.lid_tilt = sinf(animation_time_ * 5.0f) * 12.0f * remaining;
    return;
  }

  if (now >= reaction_until_) return;

  switch (reaction_) {
    case Reaction::Brake:
      target_.eye_width = 41.0f;
      target_.eye_height = 45.0f;
      target_.top_lid = face_anim::BRAKE_NARROW_LID_PX;
      target_.bottom_lid = 8.0f;
      target_.lid_tilt = 12.0f;
      target_.face_y = 2.0f;
      break;
    case Reaction::Accelerate:
      target_.eye_width = 43.0f;
      target_.eye_height = 34.0f;
      target_.top_lid = 10.0f;
      target_.bottom_lid = 5.0f;
      target_.lid_tilt = -7.0f;
      target_.face_y = -2.0f;
      break;
    case Reaction::Corner:
      target_.rotation = turn_direction_ * face_anim::TURN_ROTATION_DEG *
                         (0.7f + 0.3f * event_intensity_);
      target_.face_x = turn_direction_ * (7.0f + 4.0f * event_intensity_);
      target_.lid_tilt = turn_direction_ * 6.0f;
      target_.eye_asymmetry = turn_direction_ * 8.0f;
      break;
    case Reaction::Bump:
      target_.eye_width = 37.0f;
      target_.eye_height = 52.0f;
      target_.eye_radius = 11.0f;
      target_.face_y += sinf(animation_time_ * 34.0f) * jiggle_;
      break;
    default:
      break;
  }
}

void FaceEngine::update(uint32_t now, float forwardG, float lateralG,
                        bool driving) {
  float dt = (now - last_ms_) / 1000.0f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.033f;
  last_ms_ = now;
  animation_time_ += dt;

  tension_ = fmaxf(0.0f, tension_ - dt / 30.0f);
  flash_ = fmaxf(0.0f, flash_ - dt * face_anim::BRAKE_FLASH_DECAY);
  jiggle_ *= powf(0.08f, dt);

  if (forwardG <= face_anim::DEAD_ACCEL_G && reaction_ != Reaction::Dead) {
    reaction_ = Reaction::Dead;
    dead_until_ = now + face_anim::DEAD_DURATION_MS;
    reaction_until_ = dead_until_;
    spin_started_ = 0;
    dizzy_until_ = 0;
    blink_until_ = 0;
  }
  if (now >= reaction_until_ && spin_started_ == 0) reaction_ = Reaction::None;
  setMoodTargets(lateralG);
  applyReactionTargets(now);

  // Layer 3: blinking coexists with mood, but yields during the short reaction.
  bool reactionActive = reaction_ != Reaction::None || spin_started_ != 0;
  if (!reactionActive && now >= next_blink_ && blink_until_ == 0)
    blink_until_ = now + face_anim::BLINK_DURATION_MS;
  if (blink_until_ != 0) {
    target_.top_lid = target_.eye_height * 0.72f;
    target_.bottom_lid = target_.eye_height * 0.28f;
    if (now >= blink_until_) {
      blink_until_ = 0;
      next_blink_ = now + randomRange(face_anim::BLINK_MIN_MS,
                                      face_anim::BLINK_MAX_MS);
    }
  }

  if (!reactionActive && !driving) {
    if (now >= next_glance_ && glance_until_ == 0) {
      glance_until_ = now + randomRange(500, 1100);
      glance_direction_ = (esp_random() & 1) ? 1.0f : -1.0f;
    }
    if (glance_until_ != 0) {
      target_.face_x = glance_direction_ * face_anim::GLANCE_DISTANCE_PX;
      if (now >= glance_until_) {
        glance_until_ = 0;
        next_glance_ = now + randomRange(face_anim::GLANCE_MIN_MS,
                                         face_anim::GLANCE_MAX_MS);
      }
    }
  }

#define SPRING_FIELD(field) spring(current_.field, velocity_.field, target_.field, dt, \
                                   face_anim::SPRING_STIFFNESS, face_anim::SPRING_DAMPING)
  SPRING_FIELD(eye_width);
  SPRING_FIELD(eye_height);
  SPRING_FIELD(eye_radius);
  SPRING_FIELD(eye_asymmetry);
  SPRING_FIELD(top_lid);
  SPRING_FIELD(bottom_lid);
  SPRING_FIELD(lid_tilt);
  SPRING_FIELD(face_x);
  SPRING_FIELD(face_y);
  spring(current_.rotation, velocity_.rotation, target_.rotation, dt,
         face_anim::ROTATION_STIFFNESS, face_anim::ROTATION_DAMPING);
#undef SPRING_FIELD
}

uint16_t FaceEngine::moodBackground() const {
  return blend565(face_anim::CALM_BG, face_anim::TENSE_BG, tension_);
}

uint16_t FaceEngine::moodForeground() const {
  return blend565(face_anim::CALM_FACE, face_anim::TENSE_FACE, tension_);
}

void FaceEngine::render(TFT_eSprite& c) {
  uint16_t bg = moodBackground();
  uint16_t fg = flash_ > 0.0f
                    ? blend565(moodForeground(), face_anim::BRAKE_EYES, flash_)
                    : moodForeground();
  c.fillSprite(bg);

  const float cx = c.width() * 0.5f + current_.face_x;
  const float cy = c.height() * 0.5f + current_.face_y;
  const float rotation = current_.rotation;
  const float eyeGap = 28.0f;
  const float eyeWidth = constrain(current_.eye_width, 24.0f, 48.0f);
  const float baseHeight = constrain(current_.eye_height, 15.0f, 55.0f);
  const float radius = constrain(current_.eye_radius, 4.0f, 13.0f);

  for (int side = -1; side <= 1; side += 2) {
    float localX = side * eyeGap;
    float eyeHeight = constrain(baseHeight + side * current_.eye_asymmetry,
                                12.0f, 57.0f);
    float localY = (baseHeight - eyeHeight) * 0.16f;
    if (reaction_ == Reaction::Dead) {
      const float inset = 3.0f;
      Point nw = transformPoint(localX - eyeWidth * 0.5f + inset,
                                localY - eyeHeight * 0.5f + inset,
                                cx, cy, rotation);
      Point ne = transformPoint(localX + eyeWidth * 0.5f - inset,
                                localY - eyeHeight * 0.5f + inset,
                                cx, cy, rotation);
      Point sw = transformPoint(localX - eyeWidth * 0.5f + inset,
                                localY + eyeHeight * 0.5f - inset,
                                cx, cy, rotation);
      Point se = transformPoint(localX + eyeWidth * 0.5f - inset,
                                localY + eyeHeight * 0.5f - inset,
                                cx, cy, rotation);
      c.drawWideLine(nw.x, nw.y, se.x, se.y, 7.0f, fg, bg);
      c.drawWideLine(ne.x, ne.y, sw.x, sw.y, 7.0f, fg, bg);
      continue;
    }

    fillRoundedRotatedRect(c, localX - eyeWidth * 0.5f,
                           localY - eyeHeight * 0.5f, eyeWidth, eyeHeight,
                           radius, cx, cy, rotation, fg);

    // Background-coloured wedges reshape the solid eyes into the expression.
    // With no pupils or mouth, silhouette, spacing and motion do all the work.
    float tilt = current_.lid_tilt;
    float innerDrop = fmaxf(0.0f, tilt);
    float outerDrop = fmaxf(0.0f, -tilt);
    if (side < 0) { float tmp = innerDrop; innerDrop = outerDrop; outerDrop = tmp; }
    float left = localX - eyeWidth * 0.5f - 2.0f;
    float right = localX + eyeWidth * 0.5f + 2.0f;
    float top = localY - eyeHeight * 0.5f;
    Point a = transformPoint(left, top - 3, cx, cy, rotation);
    Point b = transformPoint(right, top - 3, cx, cy, rotation);
    Point d = transformPoint(right,
                             top + current_.top_lid + innerDrop,
                             cx, cy, rotation);
    Point e = transformPoint(left,
                             top + current_.top_lid + outerDrop,
                             cx, cy, rotation);
    fillQuad(c, a, b, d, e, bg);

    fillRotatedRect(c, left,
                    localY + eyeHeight * 0.5f - current_.bottom_lid,
                    eyeWidth + 4.0f,
                    current_.bottom_lid + 4, cx, cy, rotation, bg);
  }

  if (flash_ > 0.0f) {
    uint16_t alert = blend565(bg, face_anim::BRAKE_EYES, flash_);
    int thickness = 1 + static_cast<int>(5.0f * flash_);
    for (int i = 0; i < thickness; ++i)
      c.drawRect(i, i, c.width() - 2 * i, c.height() - 2 * i, alert);
  }
}
