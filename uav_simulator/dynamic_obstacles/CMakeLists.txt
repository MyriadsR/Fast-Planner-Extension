cmake_minimum_required(VERSION 3.0.2)
project(dynamic_obstacles)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    pcl_ros
    pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Dynamic Cube Obstacles Publisher
add_executable(dynamic_cube_obstacles
  src/dynamic_cube_obstacles.cpp
)

target_link_libraries(dynamic_cube_obstacles
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
