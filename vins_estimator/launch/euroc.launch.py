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

    support_path = PathJoinSubstitution([
        ros2_ws_path,
        'support_files'
    ])
    
    # Define the vins_estimator node
    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        namespace='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
        }]
    )

    # Define the pose_graph node
    pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph',
        name='pose_graph',
        namespace='pose_graph',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'support_file': support_path,
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0
        }]
    )

    return LaunchDescription([
        ros2_ws_path_arg,
        vins_estimator_node,
        pose_graph_node
    ])