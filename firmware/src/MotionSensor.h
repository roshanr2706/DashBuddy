// Minimal MPU6050/MPU6500 register driver used by DashBuddy.
// Both devices expose the accel/temperature/gyro burst at 0x3B and share the
// scale selections used here. Keeping this adapter small prevents sensor-model
// details from leaking into ImuProcessor's vehicle-orientation logic.
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>

enum class MotionSensorKind : uint8_t { None, MPU6050, MPU6500 };

struct MotionSample {
  float accel_g[3] = {0, 0, 0};
  float gyro_dps[3] = {0, 0, 0};
  float temperature_c = 0;
};

class MotionSensor {
 public:
  bool begin(TwoWire& wire = Wire);
  bool read(MotionSample& sample);

  MotionSensorKind kind() const { return kind_; }
  uint8_t address() const { return address_; }
  uint8_t identity() const { return identity_; }
  const char* name() const;

 private:
  bool probe(uint8_t address);
  bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
  bool readRegister(uint8_t reg, uint8_t& value);
  bool writeRegister(uint8_t reg, uint8_t value);

  TwoWire* wire_ = nullptr;
  MotionSensorKind kind_ = MotionSensorKind::None;
  uint8_t address_ = 0;
  uint8_t identity_ = 0;
};
