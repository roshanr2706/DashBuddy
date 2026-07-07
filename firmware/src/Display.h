// ============================================================================
//  Display.h — TFT_eSPI panel + a single off-screen 16-bit framebuffer.
//
//  §7 rule: never draw primitives straight to the panel. The face is composed
//  into an off-screen canvas (a TFT_eSPI sprite, allocated in PSRAM) and
//  pushed in one transfer per frame. This wrapper owns both.
// ============================================================================
#pragma once

#include <TFT_eSPI.h>

class Display {
 public:
  // Initialises the panel and allocates the framebuffer sprite in PSRAM.
  // Returns false if the sprite could not be allocated.
  bool begin();

  // The canvas everything draws into. Draw here, then call push().
  TFT_eSprite& canvas() { return fb_; }

  // Blit the framebuffer to the panel in one DMA-friendly transfer.
  void push();

  int16_t width()  const { return w_; }
  int16_t height() const { return h_; }

 private:
  TFT_eSPI  tft_;
  TFT_eSprite fb_{&tft_};
  int16_t w_ = 0;
  int16_t h_ = 0;
};
