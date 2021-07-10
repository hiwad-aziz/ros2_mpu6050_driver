#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include <string>

class MPU6050Sensor {
 public:
  MPU6050Sensor(int bus_number = 1);
  ~MPU6050Sensor();

  enum AccelRange { ACC_2_G = 0b00, ACC_4_G = 0b01, ACC_8_G = 0b10, ACC_16_G = 0b11 };
  enum GyroRange { GYR_250_DEG_S, GYR_500_DEG_S, GYR_1000_DEG_S, GYR_2000_DEG_S };

  void printConfig();
  void setGyroscopeRange(GyroRange range);
  void setAccelerometerRange(AccelRange range);
  void getAccelerations();
  void getAngularVelocities();
  void setGyroscopeOffset();
  void setAccelerometerOffset();
  void wakeUp();
  void sleep();

 private:
  void readRawAccelerometerData();
  void readRawGyroscopeData();
  void convertRawGyroscopeDate();
  void convertRawAccelerometerData();
  void reportError(int error);

  int file_;
  char filename_[10] = "/dev/i2c-";

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

  // To check
  static constexpr int SMPLRT_DIV = 0x19;
  static constexpr int CONFIG = 0x1A;
  static constexpr int INT_ENABLE = 0x38;
};

#endif  // MPU6050SENSOR_H
