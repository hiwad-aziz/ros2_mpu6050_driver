#include "mpu6050driver/mpu6050sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class MPU6050Driver : public rclcpp::Node {
 public:
  MPU6050Driver();

  // This construct of static members is required because wiringPiISR can't
  // receive a non-static member function using std::bind. Static member
  // functions do not have a this parameter so we have to keep a copy of a
  // pointer to itself after creating an instance to be able to use member
  // functions in the callback.
  // See also: https://isocpp.org/wiki/faq/pointers-to-members#memfnptr-vs-fnptr

 private:
  void handleInput();
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  size_t count_;
  std::unique_ptr<MPU6050Sensor> mpu6050_;
};