// ============================================================================
//  Buzzer.h — optional passive piezo for the hard-brake alert tone (§2, §9).
//  Non-blocking: chirp() starts a tone, update() stops it when it expires.
//  Compiles to no-ops when buzzer::ENABLED is false.
// ============================================================================
#pragma once

#include <stdint.h>

class Buzzer {
 public:
  void begin();
  void chirp(uint32_t freq_hz, uint16_t ms);
  void update(uint32_t now);

 private:
  uint32_t off_at_ = 0;
  bool     active_ = false;
};
