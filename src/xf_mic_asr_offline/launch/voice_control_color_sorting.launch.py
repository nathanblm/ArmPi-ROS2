import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ['need_compile']
    start = LaunchConfiguration('start', default='false')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    debug = LaunchConfiguration('debug', default='false')
    debug_arg = DeclareLaunchArgument('debug', default_value=debug)
    if compiled == 'True':
        app_package_path = get_package_share_directory('app')
        sdk_package_path = get_package_share_directory('sdk')
        example_package_path = get_package_share_directory('example')
        peripherals_package_path = get_package_share_directory('peripherals')
        xf_mic_asr_offline_package_path = get_package_share_directory('xf_mic_asr_offline')
    else:
        app_package_path = '/home/ubuntu/ros2_ws/src/app' 
        sdk_package_path = '/home/ubuntu/ros2_ws/src/driver/sdk'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        xf_mic_asr_offline_package_path = '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline'

    color_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(example_package_path, 'example/opencv/color_detect_node.launch.py')),
        launch_arguments={
            'enable_roi_display': debug,
        }.items(),
    )
    mic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xf_mic_asr_offline_package_path, 'launch/mic_init.launch.py')),
    )
    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/jetarm_sdk.launch.py')),
    )
    color_sorting_node = Node(
        package='example',
        executable='color_sorting',
        output='screen',
        parameters=[os.path.join(example_package_path, 'config/color_sorting_roi.yaml'), ('/home/ubuntu/ros2_ws/src/app/config/config.yaml'), {'start': start, 'debug': debug}]
    )
    voice_control_color_sorting_node = Node(
        package='xf_mic_asr_offline',
        executable='voice_control_color_sorting.py',
        output='screen',
    )

    return [ 
            sdk_launch,
            color_detect_launch,
            color_sorting_node,
            mic_launch,
            voice_control_color_sorting_node,
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
