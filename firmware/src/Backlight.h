// ============================================================================
//  Backlight.h — PWM control of the LCD backlight (BLK) via LEDC.
//
//  Dimming the backlight is the correct way to dim an LCD (§7/§11) — you can't
//  lower per-pixel brightness like an OLED. AutoDim drives setDuty() from lux.
// ============================================================================
#pragma once

#include <stdint.h>

class Backlight {
 public:
  void begin();

  // Set raw duty (0..255). Values below config MIN_DUTY are clamped up so the
  // screen never goes fully dark while the device is awake.
  void setDuty(uint8_t duty);

  uint8_t duty() const { return duty_; }

 private:
  uint8_t duty_ = 0;
};
