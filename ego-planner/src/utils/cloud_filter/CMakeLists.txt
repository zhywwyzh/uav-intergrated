cmake_minimum_required(VERSION 3.0.2)
project(cloud_filter)

## 编译选项
add_compile_options(-std=c++17)

## ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  message_filters
  geometry_msgs
  quadrotor_msgs
  tf
)

## 其他依赖
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## catkin 配置
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs pcl_ros pcl_conversions message_filters geometry_msgs quadrotor_msgs tf
)

## 头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 可执行文件 1：pointcloud_filter_node
add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
set_target_properties(pointcloud_filter_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)
target_link_libraries(pointcloud_filter_node
  ${catkin_LIBRARIES}
)

## 可执行文件 2：timestamp_converter_node
add_executable(timestamp_converter_node src/timestamp_converter_node.cpp)
set_target_properties(timestamp_converter_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)
target_link_libraries(timestamp_converter_node
  ${catkin_LIBRARIES}
)
