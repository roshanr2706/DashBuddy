#include "Backlight.h"

#include <Arduino.h>

#include "config.h"

void Backlight::begin() {
  ledcSetup(backlight::LEDC_CHANNEL, backlight::LEDC_FREQ_HZ,
            backlight::LEDC_RES_BITS);
  ledcAttachPin(pins::DISPLAY_BLK, backlight::LEDC_CHANNEL);
  setDuty(backlight::BOOT_DUTY);
}

void Backlight::setDuty(uint8_t duty) {
  if (duty < backlight::MIN_DUTY) duty = backlight::MIN_DUTY;
  duty_ = duty;
  ledcWrite(backlight::LEDC_CHANNEL, duty_);
}
