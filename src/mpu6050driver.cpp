#include "mpu6050driver/mpu6050driver.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

MPU6050Driver::MPU6050Driver()
    : Node("mpu6050publisher"), mpu6050_{std::make_unique<MPU6050Sensor>()}
{
  this->declare_parameter<bool>("calibrate", true);
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  mpu6050_->printConfig();
  mpu6050_->setGyroscopeRange(MPU6050Sensor::GyroRange::GYR_250_DEG_S);
  mpu6050_->setAccelerometerRange(MPU6050Sensor::AccelRange::ACC_2_G);
  // Check if we want to calibrate the sensor
  bool calibrate = true;
  this->get_parameter("calibrate", calibrate);
  if (calibrate) {
    mpu6050_->calibrate();
    mpu6050_->printOffsets();
  }
  timer_ = this->create_wall_timer(100ms, std::bind(&MPU6050Driver::handleInput, this));
}

void MPU6050Driver::handleInput()
{
  auto message = sensor_msgs::msg::Imu();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";
  message.linear_acceleration.x = mpu6050_->getAccelerationX();
  message.linear_acceleration.y = mpu6050_->getAccelerationY();
  message.linear_acceleration.z = mpu6050_->getAccelerationZ();
  message.angular_velocity.x = mpu6050_->getAngularVelocityX();
  message.angular_velocity.y = mpu6050_->getAngularVelocityY();
  message.angular_velocity.z = mpu6050_->getAngularVelocityZ();
  message.orientation.x = 0;
  message.orientation.y = 0;
  message.orientation.z = 0;
  message.orientation.w = 0;
  publisher_->publish(message);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU6050Driver>());
  rclcpp::shutdown();
  return 0;
}