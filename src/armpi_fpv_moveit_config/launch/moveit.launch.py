"""Run MoveIt 2 against the ArmPi-FPV hardware trajectory controllers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    use_camera = LaunchConfiguration('use_camera')
    serial_port = LaunchConfiguration('serial_port')

    moveit_config = (
        MoveItConfigsBuilder(
            'armpi_fpv_description',
            package_name='armpi_fpv_moveit_config',
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    bringup_path = os.path.join(
        get_package_share_directory('armpi_fpv_bringup'),
        'launch',
        'hardware.launch.py',
    )
    rviz_config = os.path.join(
        get_package_share_directory('armpi_fpv_moveit_config'),
        'launch',
        'moveit.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_path),
            launch_arguments={
                'use_hardware': use_hardware,
                'use_camera': use_camera,
                'use_kinematics': 'false',
                'use_web_video': 'false',
                'serial_port': serial_port,
            }.items(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(use_hardware),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_armpi_base',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'base_footprint',
            ],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[moveit_config.to_dict()],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='moveit_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
            ],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
