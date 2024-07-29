from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the launch arguments
    ros2_ws_path_arg = DeclareLaunchArgument(
        'ros2_ws_path',
        default_value='/home/db/ros2_ws/vins_ws/src/VINS-MONO-ROS2',
        description='Path to the ros2 ws'
    )

    ros2_ws_path = LaunchConfiguration('ros2_ws_path')

    config_path = PathJoinSubstitution([
        ros2_ws_path,
        'config/euroc/euroc_config.yaml'
    ])

    vins_path = PathJoinSubstitution([
        ros2_ws_path,
        'config/../'
    ])

    # Define the node
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )

    return LaunchDescription([
        ros2_ws_path_arg,
        feature_tracker_node
    ])