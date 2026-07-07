// ============================================================================
//  AutoDim.h — VEML7700 ambient light → smoothed backlight PWM (§7, Phase 4).
//
//  Low gain (1/8) + short integration time so direct windshield sun doesn't
//  saturate the sensor. Lux is mapped to a duty cycle and eased so the screen
//  fades rather than steps between light levels.
// ============================================================================
#pragma once

#include <Adafruit_VEML7700.h>
#include <stdint.h>

class Backlight;

class AutoDim {
 public:
  bool begin();

  // Call every loop; internally rate-limited. Reads lux and nudges the
  // backlight toward the target duty.
  void update(uint32_t now, Backlight& bl);

  float lux() const { return lux_; }

 private:
  Adafruit_VEML7700 veml_;
  bool ok_ = false;
  uint32_t last_read_ = 0;
  float lux_ = 0.0f;
  float duty_f_ = -1.0f;   // eased duty as float; -1 = uninitialised
};
