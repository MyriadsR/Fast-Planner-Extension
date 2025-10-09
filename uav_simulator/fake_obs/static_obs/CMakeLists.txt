cmake_minimum_required(VERSION 3.10.0)
project(static_obs)

ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_BUILD_TYPE "Release")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  nav_msgs
  roslib
  std_srvs
  obj_state_msgs
)

catkin_package(
  INCLUDE_DIRS src
  DEPENDS OpenCV Eigen Boost
  roscpp
  sensor_msgs
)

include_directories(SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(
  fake_static_obs_sim
  src/fake_static_obs.cpp
)

target_link_libraries(fake_static_obs_sim ${catkin_LIBRARIES})
