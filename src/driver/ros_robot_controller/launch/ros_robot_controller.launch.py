"""Launch the Hiwonder STM32 controller with configurable serial settings."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('serial_timeout', default_value='5.0'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),
        Node(
            package='ros_robot_controller',
            executable='ros_robot_controller',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'timeout': LaunchConfiguration('serial_timeout'),
                'imu_frame': LaunchConfiguration('imu_frame'),
            }],
        ),
    ])
