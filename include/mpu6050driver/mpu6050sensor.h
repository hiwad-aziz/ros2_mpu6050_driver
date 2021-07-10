#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include <string>

class MPU6050Sensor {
 public:
  MPU6050Sensor(int bus_number = 1);
  ~MPU6050Sensor();

  enum AccelRange { ACC_2_G, ACC_4_G, ACC_8_G, ACC_16_G };
  enum GyroRange { GYR_250_DEG_S, GYR_500_DEG_S, GYR_1000_DEG_S, GYR_2000_DEG_S };

  void printConfig();
  void setGyroscopeRange(GyroRange range);
  void setAccelerometerRange(AccelRange range);
  double getAccelerationX();
  double getAccelerationY();
  double getAccelerationZ();
  double getAngularVelocityX();
  double getAngularVelocityY();
  double getAngularVelocityZ();
  void setGyroscopeOffset();
  void setAccelerometerOffset();

 private:
  double convertRawGyroscopeData(int gyro_raw_);
  double convertRawAccelerometerData(int accel_raw_);
  void reportError(int error);

  int file_;
  char filename_[10] = "/dev/i2c-";
  int accel_range_{2};
  int gyro_range_{250};

  // MPU6050 registers and addresses (s. datasheet for details)
  static constexpr int MPU6050_ADDRESS_DEFAULT = 0x68;
  static constexpr int PWR_MGMT_1 = 0x6B;
  static constexpr int GYRO_CONFIG = 0x1B;
  static constexpr int ACCEL_CONFIG = 0x1C;
  static constexpr int ACCEL_XOUT_H = 0x3B;
  static constexpr int ACCEL_YOUT_H = 0x3D;
  static constexpr int ACCEL_ZOUT_H = 0x3F;
  static constexpr int GYRO_XOUT_H = 0x43;
  static constexpr int GYRO_YOUT_H = 0x45;
  static constexpr int GYRO_ZOUT_H = 0x47;
  // Helper constants
  static constexpr int GYRO_CONFIG_SHIFT = 3;
  static constexpr int ACCEL_CONFIG_SHIFT = 3;
  static constexpr int ACCEL_SENSITIVITY = 32768;
  static constexpr int GYRO_SENSITIVITY = 131;
  static constexpr double GRAVITY = 9.81;
};

#endif  // MPU6050SENSOR_H
