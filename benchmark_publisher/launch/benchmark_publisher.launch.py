from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ros2_ws_path_arg = DeclareLaunchArgument(
        'ros2_ws_path',
        default_value='/home/db/ros2_ws/vins_ws/src/VINS-MONO-ROS2',
        description='Path to the ros2 ws'
    )

    ros2_ws_path = LaunchConfiguration('ros2_ws_path')

    sequence_name_arg = DeclareLaunchArgument(
        'sequence_name',
        default_value='MH_01_easy',
        description='Sequence name for the benchmark'
    )

    sequence_name = LaunchConfiguration('sequence_name')
    
    data_path = PathJoinSubstitution([
        ros2_ws_path,
        'benchmark_publisher/config',
        sequence_name,
        'data.csv'
    ])

    log_data_path = LogInfo(msg=['Data path: ', data_path])

    benchmark_publisher_node = Node(
        package='benchmark_publisher',
        executable='benchmark_publisher',
        name='benchmark_publisher',
        namespace='benchmark_publisher',
        output='screen',
        parameters=[{
            'data_name': data_path
        }]
    )

    return LaunchDescription([
        ros2_ws_path_arg,
        sequence_name_arg,
        log_data_path,
        benchmark_publisher_node,
    ])