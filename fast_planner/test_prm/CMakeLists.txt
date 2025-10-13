cmake_minimum_required(VERSION 3.10.0)
project(test_prm)

ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
  obj_state_msgs
  t_prm
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs geometry_msgs obj_state_msgs t_prm
  DEPENDS Eigen3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(test_prm_graph_node
  src/test_prm_graph.cpp
)

target_link_libraries(test_prm_graph_node
  ${catkin_LIBRARIES}
)

add_executable(dynamic_obstacle_publisher
  src/publish_dynamic_obstacles.cpp
)

target_link_libraries(dynamic_obstacle_publisher
  ${catkin_LIBRARIES}
)

add_executable(test_prm_dynamic_simple_node
  src/test_prm_dynamic_simple.cpp
)

target_link_libraries(test_prm_dynamic_simple_node
  ${catkin_LIBRARIES}
)
