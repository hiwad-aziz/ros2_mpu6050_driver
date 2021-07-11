
# MPU6050 Driver for ROS2
This repository contains a ROS2 package that interfaces with an MPU6050 sensor over I2C. The sensor is calibrated on node startup (sensor needs to be on a plane with z-axis up and should not be moved during calibration). Calibration can be turned off in the launch file.

## Dependencies
-  libi2c-dev

## Setup
The number of iterations for calibration can be set up in `include/mpu6050driver/mpu6050sensor.h`.
Other parameters can be changed in `params/mpu6050.yaml`.

Build the package in your workspace:

    colcon build --packages-select mpu6050driver

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 launch mpu6050driver mpu6050driver_launch.py

