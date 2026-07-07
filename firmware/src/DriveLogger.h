// ============================================================================
//  DriveLogger.h — per-drive summaries on LittleFS (§6).
//
//  §6 rules honoured here: never log raw IMU samples — ImuProcessor buffers in
//  RAM and hands us one summary per drive. Each summary is a small JSON file
//  under /drives; Connectivity uploads pending files and we delete them on a
//  successful POST. Large 16 MB flash means the summaries never crowd OTA.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <time.h>

#include "ImuProcessor.h"

class DriveLogger {
 public:
  // Mounts LittleFS (formatting on first use) and recovers the sequence id.
  bool begin(const String& device_id);

  // Persist one completed drive. `started_epoch` is 0 when NTP hasn't synced;
  // the backend can fall back to upload time in that case.
  bool logDrive(const DriveStats& s, time_t started_epoch);

  // Iterate unsent summaries. Returns false when none remain. On true, `path`
  // and `json` are filled with the on-disk path and its contents.
  bool nextPending(String& path, String& json);

  // Remove a summary after a confirmed upload.
  void markUploaded(const String& path);

  uint32_t pendingCount();

 private:
  String   device_id_;
  uint32_t seq_ = 0;
};
