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
  int result = i2c_smbus_write_byte_data(file_, PWR_MGMT_1, 0);
  if (result < 0) reportError(errno);
}

MPU6050Sensor::~MPU6050Sensor() { close(file_); }

void MPU6050Sensor::printConfig()
{
  int accel_range = i2c_smbus_read_byte_data(file_, ACCEL_CONFIG);
  if (accel_range < 0) reportError(errno);
  int gyro_range = i2c_smbus_read_byte_data(file_, GYRO_CONFIG);
  if (gyro_range < 0) reportError(errno);
  accel_range = accel_range >> GYRO_CONFIG_SHIFT;
  gyro_range = gyro_range >> ACCEL_CONFIG_SHIFT;
  std::cout << "Accelerometer Range: " << accel_range << "\n";
  std::cout << "Gyroscope Range: " << gyro_range << "\n";
}

void MPU6050Sensor::setGyroscopeRange(MPU6050Sensor::GyroRange range)
{
  int result = i2c_smbus_write_byte_data(file_, GYRO_CONFIG, range << GYRO_CONFIG_SHIFT);
  if (result < 0) reportError(errno);
}

void MPU6050Sensor::setAccelerometerRange(MPU6050Sensor::AccelRange range)
{
  int result = i2c_smbus_write_byte_data(file_, ACCEL_CONFIG, range << ACCEL_CONFIG_SHIFT);
  if (result < 0) reportError(errno);
}

void MPU6050Sensor::getAccelerations() {}

void MPU6050Sensor::getAngularVelocities() {}
void MPU6050Sensor::setGyroscopeOffset() {}
void MPU6050Sensor::setAccelerometerOffset() {}
void MPU6050Sensor::wakeUp() {}
void MPU6050Sensor::sleep() {}
void MPU6050Sensor::readRawAccelerometerData() {}
void MPU6050Sensor::readRawGyroscopeData() {}
void MPU6050Sensor::convertRawGyroscopeDate() {}
void MPU6050Sensor::convertRawAccelerometerData() {}

void MPU6050Sensor::reportError(int error) { std::cerr << "Error! Errno: " << strerror(error); }