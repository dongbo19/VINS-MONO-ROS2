# VINS-MONO-ROS2
## ROS2 version of VINS-MONO

# 1. Prerequisites
* System  
  * Ubuntu 20.04  
  * ROS2 Galactic
* Libraries
  * OpenCV 4.2.0
  * [Ceres Solver](http://ceres-solver.org/installation.html) 1.14.0
  * Eigen 3.3.7
# 2. Build VINS-MONO-ROS2
Clone the repository and colcon build:  
```
cd $(PATH_TO_YOUR_ROS2_WS)/src
git clone https://github.com/dongbo19/VINS-MONO-ROS2.git
cd ..
colcon build
```
# 3. VINS-MONO-ROS2 on EuRoC datasets
## 3.1. ROS1 bag to ROS2 bag
Download [EuRoC datasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). However, the datasets are in ROS1 format. To run the code in ROS2, we need to first convert these datasets to ROS2 format. We can use [rosbags](https://pypi.org/project/rosbags/) for this purpose, which can convert ROS built-in messages between ROS1 and ROS2.  
## 3.2. Visual-inertial odometry and loop closure
To properly read the configuration files, please update the arguments in the launch files with the absolute path on your computer.  
First, modify the **'config_path'** and **'vins_path'** in two launch files:  
**_feature_tracker/launch/vins_feature_tracker.launch.py_** and **_vins_estimator/launch/euroc.launch.py_** 
```
DeclareLaunchArgument(
    'config_path',
    default_value='$(PATH_TO_YOUR_ROS2_WS)/src/VINS_MONO/config/euroc/euroc_config.yaml',
    description='Path to the config file'
),
DeclareLaunchArgument(
    'vins_path',
    default_value='$(PATH_TO_YOUR_ROS2_WS)s/src/VINS_MONO/config/../',
    description='Path to the VINS folder'
),
```
Then, modify the **'support_path'** in **_'$(PATH_TO_YOUR_ROS2_WS)/src/VINS_MONO/config/euroc/euroc_config.yaml'_**
```
#loop closure parameters
support_path: "$(PATH_TO_YOUR_ROS2_WS)/src/VINS_MONO/support_files"
```
**PS: After modifying the launch files in the corresponding packages, don't forget to run **_colcon build_** for those packages again.**  
Then we can run the launch files. Open four terminals, launch the feature_tracker, vins_estimator, rviz2, and ros2 bag. Take MH01 for example
```
ros2 launch feature_tracker vins_feature_tracker.launch.py              # for feature tracking
ros2 launch vins_estimator euroc.launch.py                              # for backend optimization and loop closure
rviz2 -d $(PATH_TO_YOUR_ROS2_WS)/src/VINS_MONO/config/vins_rviz.rviz    # for rviz2
ros2 bag play $(PATH_TO_YOUR_DATASET)/MH_01_easy                        # for ros2 bag
```
![mh05](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config/gif/vins_ros2_mh05.gif)
![v101](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config/gif/vins_ros2_v101.gif)
## 3.3. Visualize ground truch
First, take MH01 for example, modifying the **'sequence_name'** and **'data_path'** in the launch file: 
**_benchmark_publisher/launch/benchmark_publisher.launch.py_**
```
sequence_name_arg = DeclareLaunchArgument(
    'sequence_name',
    default_value='MH_01_easy',
    description='Sequence name for the benchmark'
)
    sequence_name = LaunchConfiguration('sequence_name')
    
data_path = PathJoinSubstitution([
    '$(PATH_TO_YOUR_ROS2_WS)/src/VINS_MONO/benchmark_publisher/config',
    sequence_name,
    'data.csv'
])
```
Then, open five terminals, launch the feature_tracker, vins_estimator, benchmark_mark, rviz2, and ros2 bag.
```
ros2 launch feature_tracker vins_feature_tracker.launch.py            # for feature tracking
ros2 launch vins_estimator euroc.launch.py                            # for backend optimization and loop closure
ros2 launch benchmark_publisher benchmark_publisher.launch.py         # for benchmark
rviz2 -d $(PATH_TO_YOUR_ROS2_WS)/src/VINS_MONO/config/vins_rviz.rviz  # for rviz2
ros2 bag play $(PATH_TO_YOUR_DATASET)/MH_01_easy                      # for ros2 bag
```
![mh01_benchmark](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config/gif/vins_ros2_benchmark_mh01.gif)
![mh02_benchmark](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config/gif/vins_ros2_benchmark_mh02.gif)
## 3.4. AR Demo
Download the [bag file](https://www.dropbox.com/scl/fi/q18lot4bfs1fqrctclz7b/ar_box.bag?rlkey=16yrxnwnt2fcutwwzwhlevd1n&e=1&dl=0), and then open three terminals  
```
ros2 launch ar_demo 3dm_bag.launch.py               # for featuer tracking, backend optimization and ar demo.
rviz2                                               # subscribe topics "AR_object" and "AR_image"
ros2 bag play $(PATH_TO_YOUR_DATASET)/ar_box        # for ros2 bag
```
![ar_demo](https://github.com/dongbo19/VINS-MONO-ROS2/blob/main/config/gif/vins_ros2_ar_demo.gif)
# 4. Acknowledgements
We use ros1 version of [VINS MONO](https://github.com/HKUST-Aerial-Robotics/VINS-Mono),  [ceres solver](http://ceres-solver.org/installation.html) for non-linear optimization, [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, and a generic [camera model](https://github.com/hengli/camodocal). Also, we referred to parts of the implementations from [VINS-FUSION-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2) and [vins-mono-ros2](https://github.com/hitzzq/vins-mono-ros2).

# 5. Licence
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.
