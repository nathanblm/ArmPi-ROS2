"""Launch the ArmPi-FPV joint and trajectory controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('servo_controller')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        Node(
            package='servo_controller',
            executable='servo_controller',
            output='screen',
            parameters=[
                os.path.join(package_path, 'config', 'servo_controller.yaml'),
                {'base_frame': base_frame},
            ],
        ),
    ])
