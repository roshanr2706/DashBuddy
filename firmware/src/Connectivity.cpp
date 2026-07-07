#include "Connectivity.h"

#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <WiFi.h>

#include "DriveLogger.h"
#include "config.h"

void Connectivity::begin() {
  // Device id from the MAC — stable across reboots, unique per board.
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char id[24];
  snprintf(id, sizeof(id), "dashbuddy-%02x%02x%02x", mac[3], mac[4], mac[5]);
  device_id_ = id;

  if (strlen(net::WIFI_SSID) == 0) {
    log_w("WIFI_SSID empty — running offline; drives will queue in flash");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(net::OTA_HOSTNAME);
  startWifi();
}

void Connectivity::startWifi() {
  log_i("WiFi connecting to \"%s\"", net::WIFI_SSID);
  WiFi.begin(net::WIFI_SSID, net::WIFI_PASSWORD);
  last_reconnect_ = millis();
}

bool Connectivity::online() const {
  return WiFi.status() == WL_CONNECTED;
}

time_t Connectivity::epochNow() const {
  if (!time_synced_) return 0;
  time_t t = time(nullptr);
  return t > 100000 ? t : 0;
}

void Connectivity::update(uint32_t now, DriveLogger& logger, bool driving) {
  if (strlen(net::WIFI_SSID) == 0) return;  // offline mode

  const bool up = online();

  // Reconnect supervision.
  if (!up && (now - last_reconnect_) >= net::RECONNECT_INTERVAL_MS) {
    log_i("WiFi retry");
    WiFi.disconnect();
    startWifi();
  }

  // Just came online: start NTP + OTA once.
  if (up && !was_online_) {
    log_i("WiFi up: %s", WiFi.localIP().toString().c_str());
    configTime(net::GMT_OFFSET_S, net::DST_OFFSET_S, net::NTP_SERVER);

    if (!ota_ready_) {
      ArduinoOTA.setHostname(net::OTA_HOSTNAME);
      ArduinoOTA.setPassword(net::OTA_PASSWORD);
      ArduinoOTA.onStart([]() { log_i("OTA start"); });
      ArduinoOTA.onEnd([]() { log_i("OTA end"); });
      ArduinoOTA.onError([](ota_error_t e) { log_e("OTA error %u", e); });
      ArduinoOTA.begin();
      ota_ready_ = true;
    }
  }
  was_online_ = up;

  if (up) {
    if (!time_synced_) {
      time_t t = time(nullptr);
      if (t > 100000) {  // NTP has populated the clock
        time_synced_ = true;
        log_i("NTP synced: %ld", (long)t);
      }
    }

    // OTA and uploads only while parked — both can block the loop for seconds
    // (or minutes, for a flash), and the face must never stutter mid-drive.
    if (!driving) {
      ArduinoOTA.handle();

      if ((now - last_upload_) >= net::UPLOAD_INTERVAL_MS) {
        last_upload_ = now;
        tryUpload(logger);
      }
    }
  }
}

void Connectivity::tryUpload(DriveLogger& logger) {
  String path, json;
  // Drain a few per cycle so a backlog clears without blocking the loop long.
  for (int i = 0; i < 4; ++i) {
    if (!logger.nextPending(path, json)) return;  // nothing left

    HTTPClient http;
    if (!http.begin(net::UPLOAD_URL)) {
      log_e("http.begin failed");
      return;
    }
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Device-Key", net::DEVICE_KEY);
    http.setTimeout(4000);
    int code = http.POST(json);
    http.end();

    if (code >= 200 && code < 300) {
      logger.markUploaded(path);
      log_i("Uploaded %s (%d)", path.c_str(), code);
    } else {
      log_w("Upload failed %s (%d) — will retry later", path.c_str(), code);
      return;  // stop; try again next interval
    }
  }
}
