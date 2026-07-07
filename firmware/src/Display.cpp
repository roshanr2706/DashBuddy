#include "Display.h"

#include <Arduino.h>

#include "config.h"

bool Display::begin() {
  tft_.init();
  tft_.setRotation(display::ROTATION);
  tft_.fillScreen(TFT_BLACK);

  w_ = display::WIDTH;
  h_ = display::HEIGHT;

  // Keep the framebuffer in PSRAM (8 MB) rather than crowding main SRAM (§7).
  fb_.setColorDepth(16);
  fb_.setAttribute(PSRAM_ENABLE, true);
  void* buf = fb_.createSprite(w_, h_);
  if (buf == nullptr) {
    log_e("Framebuffer sprite allocation failed (%dx%d)", w_, h_);
    return false;
  }
  fb_.fillSprite(TFT_BLACK);
  return true;
}

void Display::push() {
  // One transfer per frame — the whole composed face at once.
  fb_.pushSprite(0, 0);
}
