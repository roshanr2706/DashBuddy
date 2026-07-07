// ============================================================================
//  Connectivity.h — WiFi station, NTP time, summary upload, and OTA (§6).
//
//  Non-blocking: begin() kicks off an async WiFi connect; update() services
//  reconnects, NTP sync, ArduinoOTA, and drains pending drive summaries to the
//  backend. If WIFI_SSID is empty the device runs fully offline and just keeps
//  logging — summaries upload later once credentials/URL are set and reachable.
//
//  Uploads and OTA are DEFERRED while a drive is in progress: an HTTP POST can
//  block for seconds on a flaky connection and an OTA flash blocks entirely —
//  neither may ever freeze the face mid-drive. Both run once the car is parked
//  (which is also when home WiFi is actually in range).
// ============================================================================
#pragma once

#include <Arduino.h>
#include <time.h>

class DriveLogger;

class Connectivity {
 public:
  void begin();
  void update(uint32_t now, DriveLogger& logger, bool driving);

  bool   online() const;
  bool   timeSynced() const { return time_synced_; }

  // Epoch seconds, or 0 if NTP hasn't synced yet.
  time_t epochNow() const;

  // Stable per-device id derived from the WiFi MAC, e.g. "dashbuddy-a1b2c3".
  const String& deviceId() const { return device_id_; }

 private:
  void startWifi();
  void tryUpload(DriveLogger& logger);

  String device_id_;
  bool   ota_ready_   = false;
  bool   time_synced_ = false;
  bool   was_online_  = false;
  uint32_t last_reconnect_ = 0;
  uint32_t last_upload_    = 0;
};
