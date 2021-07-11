#include "mpu6050driver/mpu6050sensor.h"

extern "C" {
#include <errno.h>
#include <fcntl.h>
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
}

#include <iostream>

MPU6050Sensor::MPU6050Sensor(int bus_number)
{
  // TODO: make char append cleaner
  filename_[9] = *std::to_string(bus_number).c_str();
  std::cout << filename_ << std::endl;
  file_ = open(filename_, O_RDWR);
  if (file_ < 0) {
    std::cerr << "Failed to open file descriptor! Check your bus number! Errno: "
              << strerror(errno);
    exit(1);
  }
  if (ioctl(file_, I2C_SLAVE, MPU6050_ADDRESS_DEFAULT) < 0) {
    std::cerr << "Failed to find device address! Check device address!";
    exit(1);
  }
  // Wake up sensor
  int result = i2c_smbus_write_byte_data(file_, PWR_MGMT_1, 0);
  if (result < 0) reportError(errno);
  // Read current ranges from sensor
  readGyroscopeRange();
  readAccelerometerRange();
}

MPU6050Sensor::~MPU6050Sensor() { close(file_); }

void MPU6050Sensor::printConfig() const
{
  std::cout << "Accelerometer Range: +-" << accel_range_ << "g\n";
  std::cout << "Gyroscope Range: +-" << gyro_range_ << " degree per sec\n";
}

void MPU6050Sensor::printOffsets() const
{
  std::cout << "Accelerometer Offsets: x: " << accel_x_offset_ << ", y: " << accel_y_offset_
            << ", z: " << accel_z_offset_ << "\n";
  std::cout << "Gyroscope Offsets: x: " << gyro_x_offset_ << ", y: " << gyro_y_offset_
            << ", z: " << gyro_z_offset_ << "\n";
}

int MPU6050Sensor::readGyroscopeRange()
{
  int range = i2c_smbus_read_byte_data(file_, GYRO_CONFIG);
  if (range < 0) reportError(errno);
  range = range >> GYRO_CONFIG_SHIFT;
  gyro_range_ = GYRO_RANGES[range];
  return gyro_range_;
}

int MPU6050Sensor::readAccelerometerRange()
{
  int range = i2c_smbus_read_byte_data(file_, ACCEL_CONFIG);
  if (range < 0) reportError(errno);
  range = range >> ACCEL_CONFIG_SHIFT;
  accel_range_ = ACCEL_RANGES[range];
  return accel_range_;
}

void MPU6050Sensor::setGyroscopeRange(MPU6050Sensor::GyroRange range)
{
  int result = i2c_smbus_write_byte_data(file_, GYRO_CONFIG, range << GYRO_CONFIG_SHIFT);
  if (result < 0) reportError(errno);
  switch (range) {
    case GyroRange::GYR_250_DEG_S:
      gyro_range_ = 250;
      break;
    case GyroRange::GYR_500_DEG_S:
      gyro_range_ = 500;
      break;
    case GyroRange::GYR_1000_DEG_S:
      gyro_range_ = 1000;
      break;
    case GyroRange::GYR_2000_DEG_S:
      gyro_range_ = 2000;
      break;
    default:
      gyro_range_ = 250;
      break;
  }
}

void MPU6050Sensor::setAccelerometerRange(MPU6050Sensor::AccelRange range)
{
  int result = i2c_smbus_write_byte_data(file_, ACCEL_CONFIG, range << ACCEL_CONFIG_SHIFT);
  if (result < 0) reportError(errno);
  switch (range) {
    case AccelRange::ACC_2_G:
      accel_range_ = 2;
      break;
    case AccelRange::ACC_4_G:
      accel_range_ = 4;
      break;
    case AccelRange::ACC_8_G:
      accel_range_ = 8;
      break;
    case AccelRange::ACC_16_G:
      accel_range_ = 16;
      break;
    default:
      accel_range_ = 2;
      break;
  }
}

double MPU6050Sensor::getAccelerationX() const
{
  int accel_x_msb = i2c_smbus_read_byte_data(file_, ACCEL_XOUT_H);
  int accel_x_lsb = i2c_smbus_read_byte_data(file_, ACCEL_XOUT_H + 1);
  int accel_x = accel_x_lsb | accel_x_msb << 8;
  double accel_x_converted = convertRawAccelerometerData(accel_x);
  if (calibrated_) {
    return accel_x_converted - accel_x_offset_;
  }
  return accel_x_converted;
}

double MPU6050Sensor::getAccelerationY() const
{
  int accel_y_msb = i2c_smbus_read_byte_data(file_, ACCEL_YOUT_H);
  int accel_y_lsb = i2c_smbus_read_byte_data(file_, ACCEL_YOUT_H + 1);
  int accel_y = accel_y_lsb | accel_y_msb << 8;
  double accel_y_converted = convertRawAccelerometerData(accel_y);
  if (calibrated_) {
    return accel_y_converted - accel_y_offset_;
  }
  return accel_y_converted;
}

double MPU6050Sensor::getAccelerationZ() const
{
  int accel_z_msb = i2c_smbus_read_byte_data(file_, ACCEL_ZOUT_H);
  int accel_z_lsb = i2c_smbus_read_byte_data(file_, ACCEL_ZOUT_H + 1);
  int accel_z = accel_z_lsb | accel_z_msb << 8;
  double accel_z_converted = convertRawAccelerometerData(accel_z);
  if (calibrated_) {
    return accel_z_converted - accel_z_offset_;
  }
  return accel_z_converted;
}

double MPU6050Sensor::getAngularVelocityX() const
{
  int gyro_x_msb = i2c_smbus_read_byte_data(file_, GYRO_XOUT_H);
  int gyro_x_lsb = i2c_smbus_read_byte_data(file_, GYRO_XOUT_H + 1);
  int gyro_x = gyro_x_lsb | gyro_x_msb << 8;
  double gyro_x_converted = convertRawGyroscopeData(gyro_x);
  if (calibrated_) {
    return gyro_x_converted - gyro_x_offset_;
  }
  return gyro_x_converted;
}

double MPU6050Sensor::getAngularVelocityY() const
{
  int gyro_y_msb = i2c_smbus_read_byte_data(file_, GYRO_YOUT_H);
  int gyro_y_lsb = i2c_smbus_read_byte_data(file_, GYRO_YOUT_H + 1);
  int gyro_y = gyro_y_lsb | gyro_y_msb << 8;
  double gyro_y_converted = convertRawGyroscopeData(gyro_y);
  if (calibrated_) {
    return gyro_y_converted - gyro_y_offset_;
  }
  return gyro_y_converted;
}

double MPU6050Sensor::getAngularVelocityZ() const
{
  int gyro_z_msb = i2c_smbus_read_byte_data(file_, GYRO_ZOUT_H);
  int gyro_z_lsb = i2c_smbus_read_byte_data(file_, GYRO_ZOUT_H + 1);
  int gyro_z = gyro_z_lsb | gyro_z_msb << 8;
  double gyro_z_converted = convertRawGyroscopeData(gyro_z);
  if (calibrated_) {
    return gyro_z_converted - gyro_z_offset_;
  }
  return gyro_z_converted;
}

double MPU6050Sensor::convertRawGyroscopeData(int gyro_raw) const
{
  const double ang_vel_in_deg_per_s = static_cast<double>(gyro_raw) / GYRO_SENS_MAP.at(gyro_range_);
  return ang_vel_in_deg_per_s;
}

double MPU6050Sensor::convertRawAccelerometerData(int accel_raw) const
{
  const double accel_in_m_per_s =
      static_cast<double>(accel_raw) / ACCEL_SENS_MAP.at(accel_range_) * GRAVITY;
  return accel_in_m_per_s;
}

void MPU6050Sensor::setGyroscopeOffset(double gyro_x_offset, double gyro_y_offset,
                                       double gyro_z_offset)
{
  gyro_x_offset_ = gyro_x_offset;
  gyro_y_offset_ = gyro_y_offset;
  gyro_z_offset_ = gyro_z_offset;
}

void MPU6050Sensor::setAccelerometerOffset(double accel_x_offset, double accel_y_offset,
                                           double accel_z_offset)
{
  accel_x_offset_ = accel_x_offset;
  accel_y_offset_ = accel_y_offset;
  accel_z_offset_ = accel_z_offset;
}

void MPU6050Sensor::calibrate()
{
  int count = 0;
  while (count < CALIBRATION_COUNT) {
    gyro_x_offset_ += getAngularVelocityX();
    gyro_y_offset_ += getAngularVelocityY();
    gyro_z_offset_ += getAngularVelocityZ();
    accel_x_offset_ += getAccelerationX();
    accel_y_offset_ += getAccelerationY();
    accel_z_offset_ += getAccelerationZ();
    ++count;
  }
  gyro_x_offset_ /= CALIBRATION_COUNT;
  gyro_y_offset_ /= CALIBRATION_COUNT;
  gyro_z_offset_ /= CALIBRATION_COUNT;
  accel_x_offset_ /= CALIBRATION_COUNT;
  accel_y_offset_ /= CALIBRATION_COUNT;
  accel_z_offset_ /= CALIBRATION_COUNT;
  accel_z_offset_ -= GRAVITY;
  calibrated_ = true;
}

void MPU6050Sensor::reportError(int error) { std::cerr << "Error! Errno: " << strerror(error); }