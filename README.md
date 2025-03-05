
# MPU6050 Driver for ROS2
This repository contains a ROS2 package that interfaces with an MPU6050 sensor over I2C. The sensor is calibrated on node startup (sensor needs to be on a plane with z-axis up and should not be moved during calibration). Calibration can be turned off in the parameters file. The output is an IMU ROS message but the quaternion part is currently set to zero.

# Connecting MPU6050 to Raspberry Pi and Enabling I2C

## 1. Connect MPU6050 to Raspberry Pi (I2C)
| MPU6050 Pin | Raspberry Pi Pin |
|------------|-----------------|
| VCC        | 3.3V (Pin 1)     |
| GND        | GND (Pin 6)      |
| SCL        | GPIO3 (Pin 5)    |
| SDA        | GPIO2 (Pin 3)    |
| INT        | Not required (Optional for interrupts) |

> **Note:** MPU6050 can work with 3.3V or 5V, but it is safer to use 3.3V to avoid damage.

## 2. Enable I2C on Ubuntu
Ubuntu does not enable I2C by default. You need to manually enable it.

Open the terminal and edit the boot configuration:

```bash
sudo nano /boot/firmware/config.txt
```

Add the following lines at the end of the file:

```bash
dtparam=i2c_arm=on
```

Save and exit (CTRL+X, Y, ENTER).

Reboot the Raspberry Pi:
```bash
sudo reboot
```

## 3. Install I2C Tools and Check Connection

Install required I2C utilities:

```bash
sudo apt update
sudo apt install -y libi2c-dev i2c-tools
```

Check if the I2C bus is active:

```bash
ls /dev/i2c*
```

You should see output like:

```bash
/dev/i2c-1
```
Detect the MPU6050:

```bash
sudo i2cdetect -y 1
```
If everything is connected properly, you should see something like:

```bash
    0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

The address 0x68 confirms that the MPU6050 is connected.

## Dependencies
-  libi2c-dev

## Setup
The number of iterations for calibration can be set up in `include/mpu6050driver/mpu6050sensor.h`.
Following other parameters and default values are listed here and can be changed in `params/mpu6050.yaml`.
```    calibrate: True
    gyro_range: 0
    accel_range: 0
    dlpf_bandwidth: 2
    gyro_x_offset: 0.0 # [deg/s]
    gyro_y_offset: 0.0 # [deg/s]
    gyro_z_offset: 0.0 # [deg/s]
    accel_x_offset: 0.0 # [m/s²]
    accel_y_offset: 0.0 # [m/s²]
    accel_z_offset: 0.0 # [m/s²]
    frequency: 100 # [Hz]
```

Build the package in your workspace:

    colcon build --packages-select mpu6050driver

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 launch mpu6050driver mpu6050driver_launch.py

