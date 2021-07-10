#include "mpu6050driver/mpu6050driver.h"

#include <memory>

MPU6050Driver::MPU6050Driver()
    : Node("mpu6050publisher"), mpu6050_{std::make_unique<MPU6050Sensor>()}
{
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  mpu6050_->printConfig();
  mpu6050_->setGyroscopeRange(MPU6050Sensor::GyroRange::GYR_250_DEG_S);
  mpu6050_->setAccelerometerRange(MPU6050Sensor::AccelRange::ACC_2_G);
  mpu6050_->getAccelerations();
  mpu6050_->printConfig();
}

void MPU6050Driver::handleInput()
{
  auto message = sensor_msgs::msg::Imu();
  message.linear_acceleration.x = 0;
  message.linear_acceleration.y = 0;
  message.linear_acceleration.z = 0;
  message.angular_velocity.x = 0;
  message.angular_velocity.y = 0;
  message.angular_velocity.z = 0;
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