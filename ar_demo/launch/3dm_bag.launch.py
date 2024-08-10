from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/3dm/3dm_config.yaml'
    ])

    vins_path = PathJoinSubstitution([
        config_pkg_path,
        'config/../'
    ])
    
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='log',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
            }]
    )
    
    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        namespace='vins_estimator',
        output='log',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path
            }]
    )
    
    ar_demo_node = Node(
        package='ar_demo',
        executable='ar_demo_node',
        name='ar_demo_node',
        output='screen',
        parameters=[{'calib_file': config_path},
                    {'use_undistored_img': False}]
    )

    rviz_config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/ar_demo_rviz.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )


    return LaunchDescription([
        feature_tracker_node,
        vins_estimator_node,
        ar_demo_node,
        rviz_node
    ])
