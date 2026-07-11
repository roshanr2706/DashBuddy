// Standalone MPU6050/I2C diagnostic for the PlatformIO `displaytest` target.
// It deliberately does not use ImuProcessor, so it can distinguish wiring and
// address failures from DashBuddy orientation/event logic.
#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <math.h>

#include "config.h"
#include "MotionSensor.h"

namespace {
constexpr uint32_t kSampleIntervalMs = 100;

TFT_eSPI tft;
MotionSensor motion_sensor;
bool mpu_ok = false;
uint8_t mpu_address = 0;
uint32_t last_sample_ms = 0;

bool addressResponds(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool readRegister(uint8_t address, uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) return false;
  value = Wire.read();
  return true;
}

const char* identifyImu(uint8_t whoAmI) {
  switch (whoAmI) {
    case 0x68: return "MPU6050 / compatible";
    case 0x69: return "MPU6050 (AD0 high)";
    case 0x70: return "MPU6500-compatible variant";
    case 0x71: return "MPU9250-compatible variant";
    case 0x73: return "MPU9255-compatible variant";
    case 0x98: return "ICM20689-compatible variant";
    default: return "unknown/non-MPU6050 identity";
  }
}

void printIdentity(uint8_t address) {
  uint8_t who = 0, power = 0, accelConfig = 0;
  bool whoOk = readRegister(address, 0x75, who);       // WHO_AM_I
  bool powerOk = readRegister(address, 0x6B, power);  // PWR_MGMT_1
  bool configOk = readRegister(address, 0x1C, accelConfig);
  if (!whoOk) {
    Serial.println("  WHO_AM_I register read FAILED");
    return;
  }
  Serial.printf("  WHO_AM_I[0x75] = 0x%02X (%s)\n", who, identifyImu(who));
  if (powerOk) Serial.printf("  PWR_MGMT_1[0x6B] = 0x%02X\n", power);
  if (configOk) Serial.printf("  ACCEL_CONFIG[0x1C] = 0x%02X\n", accelConfig);
  if (who != 0x68 && who != 0x69) {
    Serial.println("  Adafruit_MPU6050 intentionally rejects this identity.");
  }
}

void drawHeader(const char* title, uint16_t color) {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(title, 4, 4, 2);
  tft.drawFastHLine(4, 23, 120, color);
}

void scanBus() {
  Serial.printf("\nI2C scan on SDA=%u SCL=%u\n", pins::I2C_SDA, pins::I2C_SCL);
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  found device at 0x%02X", address);
      if (address == 0x68 || address == 0x69) Serial.print("  <- MPU6050 candidate");
      if (address == 0x10) Serial.print("  <- VEML7700 candidate");
      Serial.println();
      ++found;
    } else if (error == 4) {
      Serial.printf("  unknown bus error at 0x%02X\n", address);
    }
  }
  if (found == 0) {
    Serial.println("  NO I2C DEVICES FOUND");
    Serial.println("  Check 3.3V/GND, swap SDA/SCL, and disconnect other devices.");
  } else {
    Serial.printf("Scan complete: %u device(s) found\n", found);
  }
}

bool connectMpu() {
  const uint8_t candidates[] = {0x68, 0x69};
  for (uint8_t address : candidates) {
    if (!addressResponds(address)) continue;
    Serial.printf("Trying MPU6050 initialization at 0x%02X...\n", address);
    printIdentity(address);
    // MotionSensor scans both addresses internally and accepts real MPU6050
    // identity 0x68 plus the common MPU6500 identity 0x70.
    if (motion_sensor.begin(Wire)) {
      mpu_address = motion_sensor.address();
      Serial.printf("%s OK at 0x%02X\n", motion_sensor.name(), mpu_address);
      return true;
    }
    Serial.printf("Address 0x%02X responds, but compatible initialization failed\n",
                  address);
    break;  // internal scan already tried both 0x68 and 0x69
  }
  return false;
}

void showFailure() {
  drawHeader("IMU NOT FOUND", TFT_RED);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("SDA: GPIO 8", 4, 33, 1);
  tft.drawString("SCL: GPIO 9", 4, 46, 1);
  tft.drawString("Expected:", 4, 64, 1);
  tft.drawString("0x68 or 0x69", 4, 77, 2);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("See serial scan", 4, 105, 1);
}

void showReading(float ax, float ay, float az, float gx, float gy, float gz,
                 float temperature) {
  drawHeader(motion_sensor.name(), TFT_GREEN);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  char line[32];
  snprintf(line, sizeof(line), "Address: 0x%02X", mpu_address);
  tft.drawString(line, 4, 28, 1);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  snprintf(line, sizeof(line), "AX %+5.2f g", ax); tft.drawString(line, 4, 42, 1);
  snprintf(line, sizeof(line), "AY %+5.2f g", ay); tft.drawString(line, 4, 54, 1);
  snprintf(line, sizeof(line), "AZ %+5.2f g", az); tft.drawString(line, 4, 66, 1);
  snprintf(line, sizeof(line), "GX %+5.0f d/s", gx); tft.drawString(line, 4, 80, 1);
  snprintf(line, sizeof(line), "GY %+5.0f d/s", gy); tft.drawString(line, 4, 92, 1);
  snprintf(line, sizeof(line), "GZ %+5.0f d/s", gz); tft.drawString(line, 4, 104, 1);
  snprintf(line, sizeof(line), "%4.1f C", temperature); tft.drawString(line, 88, 28, 1);
}
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(700);
  Serial.println("\nDashBuddy standalone IMU diagnostic");

  pinMode(pins::DISPLAY_BLK, OUTPUT);
  digitalWrite(pins::DISPLAY_BLK, HIGH);
  tft.init();
  tft.setRotation(display::ROTATION);
  drawHeader("I2C SCANNING", TFT_YELLOW);

  Wire.begin(pins::I2C_SDA, pins::I2C_SCL);
  Wire.setClock(100000);  // conservative speed for long/uncertain test wiring
  scanBus();
  mpu_ok = connectMpu();

  if (!mpu_ok) {
    showFailure();
    Serial.println("MPU6050 NOT FOUND at 0x68 or 0x69");
  } else {
    Serial.println("Move and rotate the board; values should change and reverse sign.");
  }
}

void loop() {
  if (!mpu_ok) {
    // Retry periodically so wiring can be corrected without reflashing.
    static uint32_t last_retry_ms = 0;
    if (millis() - last_retry_ms >= 3000) {
      last_retry_ms = millis();
      scanBus();
      mpu_ok = connectMpu();
      if (mpu_ok) Serial.println("MPU6050 appeared after retry.");
    }
    delay(10);
    return;
  }

  uint32_t now = millis();
  if (now - last_sample_ms < kSampleIntervalMs) return;
  last_sample_ms = now;

  MotionSample sample;
  if (!motion_sensor.read(sample)) {
    Serial.println("Motion sensor read failed; rescanning bus");
    mpu_ok = false;
    showFailure();
    return;
  }

  float ax = sample.accel_g[0];
  float ay = sample.accel_g[1];
  float az = sample.accel_g[2];
  float gx = sample.gyro_dps[0];
  float gy = sample.gyro_dps[1];
  float gz = sample.gyro_dps[2];

  Serial.printf("addr=0x%02X  accel[g] X=%+.3f Y=%+.3f Z=%+.3f  "
                "gyro[d/s] X=%+.1f Y=%+.1f Z=%+.1f  temp=%.1fC\n",
                mpu_address, ax, ay, az, gx, gy, gz, sample.temperature_c);
  showReading(ax, ay, az, gx, gy, gz, sample.temperature_c);
}
