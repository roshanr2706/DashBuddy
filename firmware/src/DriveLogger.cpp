#include "DriveLogger.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

#include "config.h"

namespace {
constexpr char kDir[] = "/drives";

String pathFor(uint32_t seq) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s/d%06u.json", kDir, seq);
  return String(buf);
}

// Normalise a directory entry to a full "/drives/..." path. Some ESP32 core
// versions return a bare basename from File::name(), others the full path.
String fullPath(const String& name) {
  if (name.startsWith("/")) return name;
  return String(kDir) + "/" + name;
}

// Basename ("d000042.json") regardless of what name()/path() returned.
String baseName(const String& name) {
  int slash = name.lastIndexOf('/');
  return slash >= 0 ? name.substring(slash + 1) : name;
}
}  // namespace

bool DriveLogger::begin(const String& device_id) {
  device_id_ = device_id;
  if (!LittleFS.begin(true /* format on failure */)) {
    log_e("LittleFS mount failed");
    return false;
  }
  if (!LittleFS.exists(kDir)) LittleFS.mkdir(kDir);

  // Recover the highest existing sequence number so ids keep increasing.
  File dir = LittleFS.open(kDir);
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    String base = baseName(f.name());  // "d000042.json"
    if (base.length() > 1 && base[0] == 'd') {
      uint32_t n = (uint32_t)strtoul(base.c_str() + 1, nullptr, 10);
      if (n >= seq_) seq_ = n + 1;
    }
    f.close();
  }
  dir.close();
  log_i("DriveLogger ready, next seq=%u", seq_);
  return true;
}

bool DriveLogger::logDrive(const DriveStats& s, time_t started_epoch) {
  JsonDocument doc;
  doc["device_id"]         = device_id_;
  doc["session_id"]        = seq_;
  doc["duration_s"]        = s.duration_s;
  doc["max_brake_g"]       = s.max_brake_g;
  doc["max_corner_g"]      = s.max_corner_g;   // peak turning (lateral) g
  doc["max_accel_g"]       = s.max_accel_g;
  doc["max_turn_dps"]      = s.max_turn_dps;   // peak turn-rate, °/s
  doc["hard_brake_count"]  = s.hard_brake_count;
  doc["hard_corner_count"] = s.hard_corner_count;
  doc["smoothness"]        = s.smoothness;
  doc["started_epoch"]     = (uint32_t)started_epoch;  // 0 if unknown
  doc["boot_start_ms"]     = s.start_ms;

  String path = pathFor(seq_);
  File f = LittleFS.open(path, FILE_WRITE);
  if (!f) {
    log_e("Could not open %s for write", path.c_str());
    return false;
  }
  serializeJson(doc, f);
  f.close();
  seq_++;
  log_i("Logged drive to %s", path.c_str());
  return true;
}

bool DriveLogger::nextPending(String& path, String& json) {
  File dir = LittleFS.open(kDir);
  bool found = false;
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    if (!f.isDirectory()) {
      path = fullPath(f.name());
      json = f.readString();
      f.close();
      found = true;
      break;
    }
    f.close();
  }
  dir.close();
  return found;
}

void DriveLogger::markUploaded(const String& path) {
  if (LittleFS.exists(path)) LittleFS.remove(path);
}

uint32_t DriveLogger::pendingCount() {
  uint32_t n = 0;
  File dir = LittleFS.open(kDir);
  for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
    if (!f.isDirectory()) n++;
    f.close();
  }
  dir.close();
  return n;
}
