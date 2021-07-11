from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    mpu6050driver_node = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[
                {"calibrate": True},
                # Gyroscope range: 0 -> +-250°/s, 1 -> +-500°/s, 2 -> +-1000°/s, 3 -> +-2000°/s
                {"gyro_range": 0},
                # Acceleration range: 0 -> +-2g, 1 -> +-4g, 2 -> +-8g, 3 -> +-16g
                {"accel_range": 0},
                # If "calibrate" is true, these values will be overriden by the calibration procedure
                {"gyro_x_offset": 0.0},   # [deg/s]
                {"gyro_y_offset": 0.0},   # [deg/s]
                {"gyro_z_offset": 0.0},   # [deg/s]
                {"accel_x_offset": 0.0},  # [m/s²]
                {"accel_y_offset": 0.0},  # [m/s²]
                {"accel_z_offset": 0.0}   # [m/s²]
        ]
    )

    ld.add_action(mpu6050driver_node)
    return ld
