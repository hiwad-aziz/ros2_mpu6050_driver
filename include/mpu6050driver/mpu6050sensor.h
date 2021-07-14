#ifndef MPU6050SENSOR_H
#define MPU6050SENSOR_H

#include <string>
#include <unordered_map>

class MPU6050Sensor {
 public:
  MPU6050Sensor(int bus_number = 1);
  ~MPU6050Sensor();

  enum AccelRange { ACC_2_G, ACC_4_G, ACC_8_G, ACC_16_G };
  enum GyroRange { GYR_250_DEG_S, GYR_500_DEG_S, GYR_1000_DEG_S, GYR_2000_DEG_S };
  enum DlpfBandwidth {
    DLPF_260_HZ,
    DLPF_184_HZ,
    DLPF_94_HZ,
    DLPF_44_HZ,
    DLPF_21_HZ,
    DLPF_10_HZ,
    DLPF_5_HZ
  };

  void printConfig() const;
  void printOffsets() const;
  void setGyroscopeRange(GyroRange range);
  void setAccelerometerRange(AccelRange range);
  void setDlpfBandwidth(DlpfBandwidth bandwidth);
  double getAccelerationX() const;
  double getAccelerationY() const;
  double getAccelerationZ() const;
  double getAngularVelocityX() const;
  double getAngularVelocityY() const;
  double getAngularVelocityZ() const;
  void setGyroscopeOffset(double gyro_x_offset, double gyro_y_offset, double gyro_z_offset);
  void setAccelerometerOffset(double accel_x_offset, double accel_y_offset, double accel_z_offset);
  void calibrate();

 private:
  double convertRawGyroscopeData(int16_t gyro_raw_) const;
  double convertRawAccelerometerData(int16_t accel_raw_) const;
  int readGyroscopeRange();
  int readAccelerometerRange();
  int readDlpfConfig();
  void reportError(int error);

  int file_;
  char filename_[10] = "/dev/i2c-";
  int accel_range_{2};
  int gyro_range_{250};
  int dlpf_range_{260};
  bool calibrated_{false};
  double gyro_x_offset_{0.0};
  double gyro_y_offset_{0.0};
  double gyro_z_offset_{0.0};
  double accel_x_offset_{0.0};
  double accel_y_offset_{0.0};
  double accel_z_offset_{0.0};

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
  static constexpr int DLPF_CONFIG = 0x1A;
  // Helper constants
  static constexpr int GYRO_CONFIG_SHIFT = 3;
  static constexpr int ACCEL_CONFIG_SHIFT = 3;
  static constexpr double GRAVITY = 9.81;
  const std::array<int, 4> ACCEL_RANGES{2, 4, 8, 16};
  const std::array<int, 4> GYRO_RANGES{250, 500, 1000, 2000};
  const std::array<int, 7> DLPF_RANGES{260, 184, 94, 44, 21, 10, 5};
  const std::unordered_map<int, int> ACCEL_SENS_MAP{{2, 16384}, {4, 8192}, {8, 4096}, {16, 2048}};
  const std::unordered_map<int, double> GYRO_SENS_MAP{
      {250, 131}, {500, 65.5}, {1000, 32.8}, {2000, 16.4}};
  static constexpr int CALIBRATION_COUNT{200};
};

#endif  // MPU6050SENSOR_H
