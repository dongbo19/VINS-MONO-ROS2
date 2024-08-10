cmake_minimum_required(VERSION 3.5)
project(benchmark_publisher)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(benchmark_publisher
    src/benchmark_publisher_node.cpp
    )

ament_target_dependencies(benchmark_publisher rclcpp rcpputils geometry_msgs nav_msgs tf2 tf2_ros)

install(
  TARGETS benchmark_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()