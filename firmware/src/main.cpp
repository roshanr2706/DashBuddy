// ============================================================================
//  Dash Companion — main firmware entry point.
//
//  Ties the subsystems together in a single non-blocking, millis()-timed loop
//  (§7). Order per frame:
//    1. IMU  — sample, classify events, track the drive session
//    2. Face — feed events, advance animation, compose + push one frame
//    3. Dim  — VEML7700 lux → backlight PWM
//    4. Log  — on drive end, write a summary to LittleFS
//    5. Net  — WiFi/NTP/OTA + drain pending summaries to the backend
//
//  Build phases (§10) map onto these modules; everything compiles without the
//  hardware attached, and missing sensors degrade gracefully (logged, skipped).
// ============================================================================
#include <Arduino.h>
#include <Wire.h>

#include "AutoDim.h"
#include "Backlight.h"
#include "Buzzer.h"
#include "Connectivity.h"
#include "Display.h"
#include "DriveLogger.h"
#include "FaceEngine.h"
#include "ImuProcessor.h"
#include "config.h"

namespace {
Backlight     g_backlight;
Display       g_display;
FaceEngine    g_face;
ImuProcessor  g_imu;
AutoDim       g_autodim;
DriveLogger   g_logger;
Connectivity  g_net;
Buzzer        g_buzzer;

bool g_display_ok = false;
bool g_imu_ok = false;

constexpr uint16_t kFrameIntervalMs = 33;  // ~30 fps face
uint32_t g_last_frame = 0;

void drawFatal(const char* msg) {
  if (!g_display_ok) return;
  TFT_eSprite& c = g_display.canvas();
  c.fillSprite(TFT_BLACK);
  c.setTextColor(TFT_RED, TFT_BLACK);
  c.setTextDatum(MC_DATUM);
  c.drawString(msg, c.width() / 2, c.height() / 2, 2);
  g_display.push();
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  log_i("Dash Companion booting");

  g_backlight.begin();

  g_display_ok = g_display.begin();
  if (!g_display_ok) log_e("Display init failed — continuing headless");

  // Shared I2C bus for MPU6050 (0x68) + VEML7700 (0x10).
  Wire.begin(pins::I2C_SDA, pins::I2C_SCL);
  Wire.setClock(400000);

  g_imu_ok = g_imu.begin();
  if (!g_imu_ok) log_e("IMU init failed — face will idle only");

  if (!g_autodim.begin()) {
    log_w("Light sensor missing — backlight stays at boot brightness");
  }

  g_buzzer.begin();
  g_face.begin();
  g_net.begin();  // computes device id even when offline

  if (!g_logger.begin(g_net.deviceId())) {
    log_e("Logger/LittleFS init failed — drives will not persist");
  }

  log_i("Device id: %s", g_net.deviceId().c_str());

  // Surface a missing IMU on the screen during bring-up — serial isn't always
  // attached in the car. The face takes over after the pause.
  if (!g_imu_ok && g_display_ok) {
    drawFatal("IMU not found");
    delay(2000);
  }

  g_last_frame = millis();
}

void loop() {
  const uint32_t now = millis();

  // --- 1. IMU + events ---
  ImuEvent ev = ImuEvent::None;
  if (g_imu_ok) {
    ev = g_imu.update();
    if (ev != ImuEvent::None) {
      FaceEventContext motion;
      motion.forward_g = g_imu.forwardG();
      motion.lateral_g = g_imu.lateralG();
      motion.vertical_g = g_imu.verticalG();
      motion.turn_rate_dps = g_imu.turnRateDps();
      g_face.onEvent(ev, motion);
      if (ev == ImuEvent::HardBrake) {
        g_buzzer.chirp(buzzer::BRAKE_TONE_HZ, buzzer::BRAKE_TONE_MS);
      }
    }

    // Phase 2 threshold-tuning stream (§10).
    if (debug::STREAM_IMU) {
      static uint32_t last = 0;
      if (now - last >= debug::STREAM_INTERVAL_MS) {
        last = now;
        Serial.printf(
            "fwd=%+.2f turnG=%+.2f vert=%+.2f turnRate=%+.0f/s "
            "cal=%d%% driving=%d\n",
            g_imu.forwardG(), g_imu.turningG(), g_imu.verticalG(),
            g_imu.turnRateDps(), (int)(g_imu.calibrationProgress() * 100),
            g_imu.driving());
      }
    }

    // --- 4. Log on drive end ---
    if (g_imu.driveJustEnded()) {
      // epochNow() is the time of drive END; back out the elapsed millis so
      // the record carries the actual START time.
      time_t started = g_net.epochNow();
      if (started > 0) {
        started -= (time_t)((millis() - g_imu.stats().start_ms) / 1000);
      }
      g_logger.logDrive(g_imu.stats(), started);
    }
  }

  g_buzzer.update(now);

  // --- 2. Face: advance + render one frame at a fixed cadence ---
  if (g_display_ok && (now - g_last_frame) >= kFrameIntervalMs) {
    g_last_frame = now;
    g_face.update(now, g_imu_ok ? g_imu.forwardG() : 0.0f,
                  g_imu_ok ? g_imu.lateralG() : 0.0f,
                  g_imu_ok ? g_imu.driving() : false);
    g_face.render(g_display.canvas());
    g_display.push();
  }

  // --- 3. Auto-dim ---
  g_autodim.update(now, g_backlight);

  // --- 5. Connectivity: WiFi/NTP/OTA + upload (deferred while driving) ---
  g_net.update(now, g_logger, g_imu_ok && g_imu.driving());
}
