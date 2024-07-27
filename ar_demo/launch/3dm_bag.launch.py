from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value='/home/db/ros2_ws/vins_ws/src/VINS_MONO/config/3dm/3dm_config.yaml',
            description='Path to config file'
        ),
        DeclareLaunchArgument(
            'vins_path',
            default_value='/home/db/ros2_ws/vins_ws/src/VINS_MONO/config/../',
            description='Path to VINS folder'
        ),
        
        Node(
            package='feature_tracker',
            executable='feature_tracker',
            name='feature_tracker',
            namespace='feature_tracker',
            output='log',
            parameters=[{'config_file': LaunchConfiguration('config_path')},
                        {'vins_folder': LaunchConfiguration('vins_path')}]
        ),
        
        Node(
            package='vins_estimator',
            executable='vins_estimator',
            name='vins_estimator',
            namespace='vins_estimator',
            output='log',
            parameters=[{'config_file': LaunchConfiguration('config_path')},
                        {'vins_folder': LaunchConfiguration('vins_path')}]
        ),
        
        Node(
            package='ar_demo',
            executable='ar_demo_node',
            name='ar_demo_node',
            output='screen',
            parameters=[{'calib_file': LaunchConfiguration('config_path')},
                        {'use_undistored_img': False}]
        )
    ])