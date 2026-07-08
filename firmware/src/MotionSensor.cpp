#include "MotionSensor.h"

namespace {
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;  // MPU6500 only
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;

constexpr float ACCEL_LSB_PER_G = 8192.0f;   // AFS_SEL=1, +/-4 g
constexpr float GYRO_LSB_PER_DPS = 65.5f;    // FS_SEL=1, +/-500 dps

int16_t signedWord(const uint8_t* bytes) {
  return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);
}
}  // namespace

bool MotionSensor::begin(TwoWire& wire) {
  wire_ = &wire;
  kind_ = MotionSensorKind::None;
  for (uint8_t address : {static_cast<uint8_t>(0x68),
                          static_cast<uint8_t>(0x69)}) {
    if (probe(address)) return true;
  }
  return false;
}

bool MotionSensor::probe(uint8_t address) {
  address_ = address;
  if (!readRegister(REG_WHO_AM_I, identity_)) return false;
  if (identity_ == 0x68) kind_ = MotionSensorKind::MPU6050;
  else if (identity_ == 0x70) kind_ = MotionSensorKind::MPU6500;
  else return false;

  // Reset, wake on the X gyro clock, then select a conservative 100 Hz,
  // 41/44 Hz filtered stream at +/-4 g and +/-500 degrees/s.
  if (!writeRegister(REG_PWR_MGMT_1, 0x80)) return false;
  delay(100);
  if (!writeRegister(REG_PWR_MGMT_1, 0x01)) return false;
  delay(10);
  if (!writeRegister(REG_CONFIG, 0x03)) return false;
  if (!writeRegister(REG_SMPLRT_DIV, 0x09)) return false;
  if (!writeRegister(REG_GYRO_CONFIG, 0x08)) return false;
  if (!writeRegister(REG_ACCEL_CONFIG, 0x08)) return false;
  if (kind_ == MotionSensorKind::MPU6500 &&
      !writeRegister(REG_ACCEL_CONFIG2, 0x03)) return false;

  uint8_t gyroConfig = 0, accelConfig = 0;
  if (!readRegister(REG_GYRO_CONFIG, gyroConfig) ||
      !readRegister(REG_ACCEL_CONFIG, accelConfig)) return false;
  return (gyroConfig & 0x18) == 0x08 && (accelConfig & 0x18) == 0x08;
}

bool MotionSensor::read(MotionSample& sample) {
  uint8_t data[14];
  if (!readRegisters(REG_ACCEL_XOUT_H, data, sizeof(data))) return false;

  for (int axis = 0; axis < 3; ++axis)
    sample.accel_g[axis] = signedWord(&data[axis * 2]) / ACCEL_LSB_PER_G;
  int16_t rawTemp = signedWord(&data[6]);
  for (int axis = 0; axis < 3; ++axis)
    sample.gyro_dps[axis] = signedWord(&data[8 + axis * 2]) / GYRO_LSB_PER_DPS;

  sample.temperature_c = kind_ == MotionSensorKind::MPU6500
      ? rawTemp / 333.87f + 21.0f
      : rawTemp / 340.0f + 36.53f;
  return true;
}

const char* MotionSensor::name() const {
  if (kind_ == MotionSensorKind::MPU6050) return "MPU6050";
  if (kind_ == MotionSensorKind::MPU6500) return "MPU6500";
  return "none";
}

bool MotionSensor::readRegisters(uint8_t reg, uint8_t* data, size_t length) {
  if (!wire_ || !address_) return false;
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  size_t received = wire_->requestFrom(address_, static_cast<uint8_t>(length));
  if (received != length) return false;
  for (size_t i = 0; i < length; ++i) data[i] = wire_->read();
  return true;
}

bool MotionSensor::readRegister(uint8_t reg, uint8_t& value) {
  return readRegisters(reg, &value, 1);
}

bool MotionSensor::writeRegister(uint8_t reg, uint8_t value) {
  if (!wire_ || !address_) return false;
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}
