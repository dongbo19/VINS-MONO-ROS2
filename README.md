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
For example, modify the **'config_path'** and **'vins_path'** in two launch files:  
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
PS: After modifying the launch files in the corresponding packages, don't forget to run **_colcon build_** for those packages again.  
Then we can run the launch files. Open four terminal, launch the feature_tracker, vins_estimator, rviz2, and ros2 bag. Take MH01 for example
```

```
