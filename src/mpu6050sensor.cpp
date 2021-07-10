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

double MPU6050Sensor::getAccelerationX()
{
  int accel_x_msb = i2c_smbus_read_byte_data(file_, ACCEL_XOUT_H);
  int accel_x_lsb = i2c_smbus_read_byte_data(file_, ACCEL_XOUT_H + 1);
  int accel_x = accel_x_lsb | accel_x_msb << 8;
  return convertRawAccelerometerData(accel_x);
}

double MPU6050Sensor::getAccelerationY()
{
  int accel_y_msb = i2c_smbus_read_byte_data(file_, ACCEL_YOUT_H);
  int accel_y_lsb = i2c_smbus_read_byte_data(file_, ACCEL_YOUT_H + 1);
  int accel_y = accel_y_lsb | accel_y_msb << 8;
  return convertRawAccelerometerData(accel_y);
}

double MPU6050Sensor::getAccelerationZ()
{
  int accel_z_msb = i2c_smbus_read_byte_data(file_, ACCEL_ZOUT_H);
  int accel_z_lsb = i2c_smbus_read_byte_data(file_, ACCEL_ZOUT_H + 1);
  int accel_z = accel_z_lsb | accel_z_msb << 8;
  return convertRawAccelerometerData(accel_z);
}

double MPU6050Sensor::getAngularVelocityX()
{
  int gyro_x_msb = i2c_smbus_read_byte_data(file_, GYRO_XOUT_H);
  int gyro_x_lsb = i2c_smbus_read_byte_data(file_, GYRO_XOUT_H + 1);
  int gyro_x = gyro_x_lsb | gyro_x_msb << 8;
  return convertRawGyroscopeData(gyro_x);
}

double MPU6050Sensor::getAngularVelocityY()
{
  int gyro_y_msb = i2c_smbus_read_byte_data(file_, GYRO_YOUT_H);
  int gyro_y_lsb = i2c_smbus_read_byte_data(file_, GYRO_YOUT_H + 1);
  int gyro_y = gyro_y_lsb | gyro_y_msb << 8;
  return convertRawGyroscopeData(gyro_y);
}

double MPU6050Sensor::getAngularVelocityZ()
{
  int gyro_z_msb = i2c_smbus_read_byte_data(file_, GYRO_ZOUT_H);
  int gyro_z_lsb = i2c_smbus_read_byte_data(file_, GYRO_ZOUT_H + 1);
  int gyro_z = gyro_z_lsb | gyro_z_msb << 8;
  return convertRawGyroscopeData(gyro_z);
}

double MPU6050Sensor::convertRawGyroscopeData(int gyro_raw_)
{
  std::cout << "Angular Velocity: "
            << static_cast<double>(gyro_range_) * gyro_raw_ / GYRO_SENSITIVITY / gyro_range_
            << std::endl;
  return static_cast<double>(gyro_range_) * gyro_raw_ / GYRO_SENSITIVITY / gyro_range_;
}
double MPU6050Sensor::convertRawAccelerometerData(int accel_raw_)
{
  std::cout << "Acceleration: "
            << static_cast<double>(accel_range_) * accel_raw_ / ACCEL_SENSITIVITY * GRAVITY
            << std::endl;
  return static_cast<double>(accel_range_) * accel_raw_ / ACCEL_SENSITIVITY * GRAVITY;
}

void MPU6050Sensor::setGyroscopeOffset() {}

void MPU6050Sensor::setAccelerometerOffset() {}

void MPU6050Sensor::reportError(int error) { std::cerr << "Error! Errno: " << strerror(error); }