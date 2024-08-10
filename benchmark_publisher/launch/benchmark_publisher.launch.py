from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    benchmark_pkg_path = get_package_share_directory('benchmark_publisher')

    sequence_name_arg = DeclareLaunchArgument(
        'sequence_name',
        default_value='MH_01_easy',
        description='Sequence name for the benchmark'
    )

    sequence_name = LaunchConfiguration('sequence_name')
    
    data_path = PathJoinSubstitution([
        benchmark_pkg_path,
        'config',
        sequence_name,
        'data.csv'
    ])

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
        benchmark_publisher_node
    ])