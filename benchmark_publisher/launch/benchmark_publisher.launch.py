from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    sequence_name_arg = DeclareLaunchArgument(
        'sequence_name',
        default_value='MH_02_easy',
        description='Sequence name for the benchmark'
    )

    sequence_name = LaunchConfiguration('sequence_name')
    
    data_path = PathJoinSubstitution([
        '/home/db/ros2_ws/vins_ws/src/VINS_MONO/benchmark_publisher/config',
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
        sequence_name_arg,
        log_data_path,
        benchmark_publisher_node,
    ])