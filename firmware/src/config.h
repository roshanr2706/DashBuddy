// ============================================================================
//  config.h — one place to tune pins, thresholds, and connectivity.
//
//  Pin assignments mirror the wiring table in §5 of the build plan. Before
//  wiring, confirm against the board silkscreen and the pin hazards in §8
//  (avoid GPIO26-37 flash/PSRAM, GPIO0/3/45/46 strapping, GPIO19/20 USB).
// ============================================================================
#pragma once

#include <stdint.h>

// ---------------------------------------------------------------------------
//  Display (SPI) — the SPI/CS/DC/RST pins are configured for TFT_eSPI via
//  build_flags in platformio.ini. Only the backlight is owned by firmware.
// ---------------------------------------------------------------------------
namespace pins {
  constexpr uint8_t DISPLAY_BLK = 15;   // PWM backlight (LEDC), to LCD BLK

  // I2C bus — shared by MPU6050 (0x68) and VEML7700 (0x10).
  constexpr uint8_t I2C_SDA = 8;
  constexpr uint8_t I2C_SCL = 9;

  constexpr uint8_t BUZZER = 2;         // optional passive piezo (PWM)
}

// ---------------------------------------------------------------------------
//  Display geometry
// ---------------------------------------------------------------------------
namespace display {
  constexpr int16_t WIDTH  = 128;
  constexpr int16_t HEIGHT = 128;
  constexpr uint8_t ROTATION = 0;
}

// ---------------------------------------------------------------------------
//  Face animation and reaction tuning
// ---------------------------------------------------------------------------
namespace face_anim {
  // Grobot-style spring response. Higher stiffness reacts faster; higher
  // damping removes bounce. Rotation is intentionally a little looser.
  constexpr float SPRING_STIFFNESS = 145.0f;
  constexpr float SPRING_DAMPING   = 19.0f;
  constexpr float ROTATION_STIFFNESS = 105.0f;
  constexpr float ROTATION_DAMPING   = 16.0f;

  constexpr uint16_t CALM_BG  = 0x0009;  // deep blue
  constexpr uint16_t TENSE_BG = 0x5800;  // dark red
  constexpr uint16_t CALM_FACE  = 0x9E7F; // cyan-white
  constexpr uint16_t TENSE_FACE = 0xFD00; // amber
  constexpr uint16_t BRAKE_EYES = 0xF800; // bright red

  // Gravity-removed forward acceleration. A value below this threshold shows
  // X eyes for the full hold duration. -1.389 m/s^2 / standard gravity.
  constexpr float DEAD_ACCEL_G = -1.389f / 9.80665f;
  constexpr uint32_t DEAD_DURATION_MS = 30000;

  constexpr float LIVE_LEAN_PX_PER_G = 18.0f;
  constexpr float MAX_LIVE_LEAN_PX = 11.0f;
  constexpr float TURN_ROTATION_DEG = 18.0f;
  constexpr float SPIN_TRIGGER_G = 0.58f;
  constexpr float SPIN_TRIGGER_DPS = 42.0f;
  constexpr uint32_t SPIN_DURATION_MS = 620;
  constexpr uint32_t DIZZY_DURATION_MS = 1450;
  constexpr float DIZZY_WOBBLE_DEG = 7.0f;
  constexpr float DIZZY_PUPIL_DRIFT_PX = 4.0f;

  constexpr uint32_t BRAKE_DURATION_MS = 360;
  constexpr float BRAKE_NARROW_LID_PX = 13.0f;
  constexpr float BRAKE_FLASH_DECAY = 8.5f;
  constexpr uint32_t ACCEL_DURATION_MS = 520;
  constexpr uint32_t CORNER_DURATION_MS = 650;
  constexpr uint32_t BUMP_DURATION_MS = 420;
  constexpr float BUMP_JIGGLE_PX = 7.0f;

  constexpr uint32_t BLINK_MIN_MS = 2200;
  constexpr uint32_t BLINK_MAX_MS = 5600;
  constexpr uint32_t BLINK_DURATION_MS = 135;
  constexpr uint32_t GLANCE_MIN_MS = 3000;
  constexpr uint32_t GLANCE_MAX_MS = 9000;
  constexpr float GLANCE_DISTANCE_PX = 7.0f;
}

// ---------------------------------------------------------------------------
//  Backlight PWM (LEDC)
// ---------------------------------------------------------------------------
namespace backlight {
  constexpr uint8_t  LEDC_CHANNEL   = 0;
  constexpr uint32_t LEDC_FREQ_HZ   = 5000;
  constexpr uint8_t  LEDC_RES_BITS  = 8;      // 0..255 duty
  constexpr uint8_t  MIN_DUTY       = 12;     // never fully off while awake
  constexpr uint8_t  MAX_DUTY       = 255;
  constexpr uint8_t  BOOT_DUTY      = 180;
}

// ---------------------------------------------------------------------------
//  Buzzer (optional). Set ENABLED=false if no piezo is fitted.
// ---------------------------------------------------------------------------
namespace buzzer {
  constexpr bool     ENABLED       = true;
  constexpr uint8_t  LEDC_CHANNEL  = 1;
  constexpr uint32_t BRAKE_TONE_HZ = 1800;
  constexpr uint16_t BRAKE_TONE_MS = 120;
}

// ---------------------------------------------------------------------------
//  IMU event thresholds (in g). §2 warning: TUNE THESE FROM REAL IN-CAR DATA.
//  Desk values will be wrong. Phase 2 streams g-values to serial so you can
//  read your car's real braking/cornering numbers and set these.
// ---------------------------------------------------------------------------
namespace debug {
  // Phase 2 (§10): stream live g-values to serial so you can read real
  // braking/cornering numbers and set thresholds from data, not a desk.
  constexpr bool     STREAM_IMU        = true;
  constexpr uint32_t STREAM_INTERVAL_MS = 50;
}

namespace imu {
  constexpr uint16_t SAMPLE_HZ = 100;

  // --- Orientation auto-detection (§7: "auto-detect the dominant horizontal
  //     axis while driving") -------------------------------------------------
  // The board's axes are figured out at runtime, so it can be mounted at any
  // angle. "Up" comes from gravity; "forward" is learned from straight-line
  // braking/accelerating; turning is measured against both.

  // Gravity estimate: slow low-pass of raw accel gives the vertical axis (s).
  // The filter is frozen while the accel vector deviates from the estimate
  // (braking/cornering) so sustained maneuvers can't tilt "up".
  constexpr float GRAVITY_TAU_S        = 2.0f;
  constexpr float GRAVITY_FREEZE_DEV_G = 0.15f;  // |acc - gravity| (g) to freeze

  // Gyro zero-rate bias: every MPU6050 reads a few °/s when perfectly still.
  // The bias is learned whenever the device is quiet and subtracted from all
  // turn-rate readings (it also drifts with temperature, so keep learning).
  constexpr float GYRO_BIAS_TAU_S   = 20.0f;   // EMA time constant while quiet
  constexpr float QUIET_ACCEL_G     = 0.05f;   // "quiet" = dyn accel below this
  constexpr float QUIET_GYRO_DPS    = 6.0f;    // ...and raw rate below this

  // Forward-axis learning: only accumulate when the car is driving roughly
  // straight (low turn-rate) with real longitudinal accel — that vector is the
  // fore-aft axis.
  constexpr float    CAL_STRAIGHT_YAW_DPS = 5.0f;   // |turn-rate| below this = straight
  constexpr float    CAL_MIN_ACCEL_G      = 0.12f;  // longitudinal accel worth learning from
  constexpr float    CAL_LEARN_RATE       = 0.02f;  // EMA toward the observed axis
  constexpr float    CAL_CONFIDENT_WEIGHT = 8.0f;   // accumulated evidence to call it "calibrated"

  // Forward sign: 0 = auto (assume braking peaks exceed acceleration peaks, so
  // the harder direction is the rear). Set +1/-1 to force it if auto guesses
  // wrong. Lateral sign flips left/right if the face leans the wrong way.
  constexpr int   FORWARD_SIGN_OVERRIDE = 0;
  constexpr float LATERAL_SIGN          = 1.0f;

  // Persist the learned calibration to NVS so it survives reboots. Bump
  // CAL_VERSION to force a re-learn after changing the mounting.
  constexpr bool     PERSIST_CALIBRATION = true;
  constexpr uint32_t CAL_VERSION         = 1;

  // A turn must exceed this turn-rate to count as cornering (rejects side
  // bumps that produce lateral g without any actual rotation).
  constexpr float CORNER_YAW_GATE_DPS = 4.0f;

  // Extra "still moving" signals for session detection, so smooth highway
  // cruising (near-zero horizontal g) doesn't end the drive: road vibration
  // (vertical) and any real turning both count as motion.
  constexpr float MOTION_VERT_G   = 0.10f;
  constexpr float MOTION_YAW_DPS  = 3.0f;

  // --- Event thresholds (g), on the gravity-removed, axis-mapped signal ------
  constexpr float BRAKE_G   = 0.45f;   // forward deceleration
  constexpr float CORNER_G  = 0.40f;   // turning (lateral) g
  constexpr float ACCEL_G   = 0.40f;   // forward acceleration
  constexpr float BUMP_G    = 0.60f;   // vertical

  // Cooldown after firing an event of a given kind (ms) — prevents strobing.
  constexpr uint32_t EVENT_COOLDOWN_MS = 700;

  // "Hard" event counters (for the drive summary) trip at these levels.
  constexpr float HARD_BRAKE_G  = 0.55f;
  constexpr float HARD_CORNER_G = 0.50f;

  // Motion/stillness detection for drive session boundaries.
  constexpr float  MOTION_G          = 0.08f;   // |dynamic accel| above this = moving
  constexpr uint32_t DRIVE_START_MS  = 3000;    // sustained motion to start a drive
  constexpr uint32_t DRIVE_END_MS    = 90000;   // stillness before a drive ends
}

// ---------------------------------------------------------------------------
//  Auto-dim (VEML7700 → backlight). §7: low gain + short integration so the
//  windshield sun doesn't saturate the sensor.
// ---------------------------------------------------------------------------
namespace autodim {
  constexpr uint32_t READ_INTERVAL_MS = 250;
  constexpr float    LUX_DARK  = 5.0f;      // <= this maps to MIN brightness
  constexpr float    LUX_BRIGHT = 800.0f;   // >= this maps to MAX brightness
  constexpr float    SMOOTHING = 0.08f;     // EMA factor toward target duty
}

// ---------------------------------------------------------------------------
//  Connectivity (§6). Secrets are NOT stored here — they come from the
//  gitignored firmware/.env, injected at build time as CFG_* defines by
//  scripts/load_env.py. Copy firmware/.env.example to firmware/.env and fill
//  it in. Missing/empty values are fine: WIFI_SSID empty = run fully offline
//  (still logs to LittleFS, uploads once configured).
// ---------------------------------------------------------------------------

// Fallbacks so the firmware always compiles even without a .env present.
#ifndef CFG_WIFI_SSID
#define CFG_WIFI_SSID ""
#endif
#ifndef CFG_WIFI_PASSWORD
#define CFG_WIFI_PASSWORD ""
#endif
#ifndef CFG_UPLOAD_URL
#define CFG_UPLOAD_URL "http://192.168.1.100:3000/api/drives"
#endif
#ifndef CFG_DEVICE_KEY
#define CFG_DEVICE_KEY "change-me"
#endif
#ifndef CFG_OTA_PASSWORD
#define CFG_OTA_PASSWORD "dashbuddy-ota"
#endif

namespace net {
  constexpr char WIFI_SSID[]     = CFG_WIFI_SSID;
  constexpr char WIFI_PASSWORD[] = CFG_WIFI_PASSWORD;

  // Backend upload endpoint (see ../backend). Include scheme + path.
  constexpr char UPLOAD_URL[]    = CFG_UPLOAD_URL;

  // Shared secret sent as X-Device-Key; must match backend DEVICE_KEY.
  constexpr char DEVICE_KEY[]    = CFG_DEVICE_KEY;

  constexpr char OTA_HOSTNAME[]  = "dashbuddy";
  constexpr char OTA_PASSWORD[]  = CFG_OTA_PASSWORD;   // ArduinoOTA password

  constexpr char NTP_SERVER[]    = "pool.ntp.org";
  constexpr long GMT_OFFSET_S    = 0;      // set your timezone offset if desired
  constexpr int  DST_OFFSET_S    = 0;

  constexpr uint32_t RECONNECT_INTERVAL_MS = 30000;
  constexpr uint32_t UPLOAD_INTERVAL_MS    = 15000;
}
