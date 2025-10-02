cmake_minimum_required(VERSION 3.11)

project(t_prm)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  nav_msgs
  geometry_msgs
  obj_state_msgs
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find OpenMP
find_package(OpenMP REQUIRED)

# Catkin package configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS EIGEN3 OpenMP
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Collect source and header files
file(GLOB TPRM_SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE TPRM_HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

# Build the library
add_library(${PROJECT_NAME} ${TPRM_SRC_FILES} ${TPRM_HEADER_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

# Build ROS example node
add_executable(ros_dynamic_test examples/ros_dynamic_test.cpp)

target_link_libraries(ros_dynamic_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_include_directories(ros_dynamic_test PRIVATE
  ${catkin_INCLUDE_DIRS}
)

# Optional: Build examples
option(BUILD_EXAMPLES "Build example programs" ON)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# Optional: Build benchmarking
option(BUILD_BENCHMARKING "Build benchmarking" OFF)
if(BUILD_BENCHMARKING)
  add_subdirectory(benchmarking)
endif()

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the ROS example node
install(TARGETS ros_dynamic_test
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
