#include "Buzzer.h"

#include <Arduino.h>

#include "config.h"

void Buzzer::begin() {
  if (!buzzer::ENABLED) return;
  ledcSetup(buzzer::LEDC_CHANNEL, 2000, 10);
  ledcAttachPin(pins::BUZZER, buzzer::LEDC_CHANNEL);
  ledcWrite(buzzer::LEDC_CHANNEL, 0);
}

void Buzzer::chirp(uint32_t freq_hz, uint16_t ms) {
  if (!buzzer::ENABLED) return;
  ledcWriteTone(buzzer::LEDC_CHANNEL, freq_hz);
  ledcWrite(buzzer::LEDC_CHANNEL, 512);  // 50% duty on 10-bit
  off_at_ = millis() + ms;
  active_ = true;
}

void Buzzer::update(uint32_t now) {
  if (!buzzer::ENABLED || !active_) return;
  if (now >= off_at_) {
    ledcWrite(buzzer::LEDC_CHANNEL, 0);
    active_ = false;
  }
}
