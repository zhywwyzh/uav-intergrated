cmake_minimum_required(VERSION 3.0.2)
project(mission_fsm)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    sensor_msgs
    pcl_conversions
    std_msgs
    roslib
    geometry_msgs
    quadrotor_msgs
    cv_bridge
    ego_planner
    map_interface
    mavros
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES exploration_manager
    CATKIN_DEPENDS ego_planner  # lkh_tsp_solver
    #  DEPENDS system_lib
)

include_directories(
        include
        SYSTEM
        ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
)

add_executable(mission_fsm_node
    src/mission_node.cpp
    src/rc_fsm.cpp
)

target_link_libraries(mission_fsm_node
    ${catkin_LIBRARIES}
)

add_executable(lidar_position_filter_node
    src/lidar_position_filter_node.cpp
)

target_link_libraries(lidar_position_filter_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
