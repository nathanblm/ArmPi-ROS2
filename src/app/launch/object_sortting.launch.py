import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        example_package_path = get_package_share_directory('example')
        app_package_path = get_package_share_directory('app')
    else:
        example_package_path = '/home/ubuntu/ros2_ws/src/example'
        app_package_path = '/home/ubuntu/ros2_ws/src/app'



    object_sortting_node = Node(
        package='app',
        executable='object_sortting',
        output='screen',
        parameters=[ os.path.join(app_package_path, 'config/config.yaml')],
        # parameters=[ os.path.join(app_package_path, 'config/config.yaml'),os.path.join(app_package_path, 'config/positions.yaml')],
    )

    return [
            object_sortting_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
