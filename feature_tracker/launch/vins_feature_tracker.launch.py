from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'config_path',
            default_value='/home/db/ros2_ws/vins_ws/src/VINS_MONO/config/euroc/euroc_config.yaml',
            description='Path to the config file'
        ),
        DeclareLaunchArgument(
            'vins_path',
            default_value='/home/db/ros2_ws/vins_ws/src/VINS_MONO/config/../',
            description='Path to the VINS folder'
        ),
        
        # Define the node
        Node(
            package='feature_tracker',
            executable='feature_tracker',
            name='feature_tracker',
            namespace='feature_tracker',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_path'),
                'vins_folder': LaunchConfiguration('vins_path')
            }]
        )
    ])