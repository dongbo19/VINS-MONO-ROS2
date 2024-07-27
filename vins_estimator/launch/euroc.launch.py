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
        
        # Define the vins_estimator node
        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            namespace='vins_estimator',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_path'),
                'vins_folder': LaunchConfiguration('vins_path')
            }]
        ),

        # Define the pose_graph node
        Node(
            package='pose_graph',
            executable='pose_graph',
            name='pose_graph',
            namespace='pose_graph',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_path'),
                'visualization_shift_x': 0,
                'visualization_shift_y': 0,
                'skip_cnt': 0,
                'skip_dis': 0.0
            }]
        )
    ])