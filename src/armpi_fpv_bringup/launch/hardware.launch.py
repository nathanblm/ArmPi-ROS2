"""Start the ArmPi-FPV ROS 2 hardware stack."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_launch(package_name, launch_file, **arguments):
    path = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_file,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    camera_device = LaunchConfiguration('camera_device')
    use_hardware = LaunchConfiguration('use_hardware')
    use_camera = LaunchConfiguration('use_camera')
    use_kinematics = LaunchConfiguration('use_kinematics')
    use_web_video = LaunchConfiguration('use_web_video')

    camera_config = os.path.join(
        get_package_share_directory('armpi_fpv_bringup'),
        'config',
        'usb_cam.yaml',
    )

    hardware_group = GroupAction(
        condition=IfCondition(use_hardware),
        actions=[
            package_launch(
                'ros_robot_controller',
                'ros_robot_controller.launch.py',
                serial_port=serial_port,
                baudrate=baudrate,
            ),
            TimerAction(
                period=1.0,
                actions=[package_launch(
                    'servo_controller',
                    'servo_controller.launch.py',
                    base_frame='base_footprint',
                )],
            ),
            TimerAction(
                period=2.0,
                actions=[package_launch(
                    'kinematics',
                    'kinematics_node.launch.py',
                )],
                condition=IfCondition(use_kinematics),
            ),
        ],
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[camera_config, {'video_device': camera_device}],
        remappings=[
            ('image_raw', '/usb_cam/rgb/image_raw'),
            ('camera_info', '/usb_cam/rgb/camera_info'),
        ],
        condition=IfCondition(use_camera),
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('use_hardware', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_kinematics', default_value='true'),
        DeclareLaunchArgument('use_web_video', default_value='false'),
        package_launch(
            'armpi_fpv_description',
            'description.launch.py',
            use_sim_time='false',
        ),
        hardware_group,
        camera_node,
        Node(
            package='web_video_server',
            executable='web_video_server',
            output='screen',
            condition=IfCondition(use_web_video),
        ),
    ])
