#include "AutoDim.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "Backlight.h"
#include "config.h"

bool AutoDim::begin() {
  ok_ = veml_.begin(&Wire);
  if (!ok_) {
    log_e("VEML7700 not found on I2C 0x10");
    return false;
  }
  // Low gain + short integration so bright sun doesn't peg the reading (§7).
  veml_.setGain(VEML7700_GAIN_1_8);
  veml_.setIntegrationTime(VEML7700_IT_25MS);
  return true;
}

void AutoDim::update(uint32_t now, Backlight& bl) {
  if (!ok_) return;
  if ((uint32_t)(now - last_read_) < autodim::READ_INTERVAL_MS) return;
  last_read_ = now;

  lux_ = veml_.readLux();

  // Map lux (log-ish perceptual scale) to 0..1 brightness.
  float lo = logf(autodim::LUX_DARK + 1.0f);
  float hi = logf(autodim::LUX_BRIGHT + 1.0f);
  float x = logf(fmaxf(lux_, 0.0f) + 1.0f);
  float f = (x - lo) / (hi - lo);
  if (f < 0) f = 0;
  if (f > 1) f = 1;

  float target = backlight::MIN_DUTY +
                 f * (backlight::MAX_DUTY - backlight::MIN_DUTY);

  if (duty_f_ < 0) duty_f_ = target;  // snap on first read
  duty_f_ += (target - duty_f_) * autodim::SMOOTHING;

  bl.setDuty((uint8_t)lroundf(duty_f_));
}
