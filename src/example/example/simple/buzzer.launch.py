import os
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    compiled = os.environ['need_compile']

    if compiled == 'True':
        robot_controller_package_path = get_package_share_directory('ros_robot_controller')
    else:
        robot_controller_package_path = '/home/ubuntu/ros2_ws/src/driver/ros_robot_controller'


    robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_controller_package_path, 'launch/ros_robot_controller.launch.py')
        ]),
    )

    buzzer_node = Node(
        package='example',
        executable='buzzer',
        output='screen',
            )

    return LaunchDescription([
        robot_controller_launch,
        buzzer_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
