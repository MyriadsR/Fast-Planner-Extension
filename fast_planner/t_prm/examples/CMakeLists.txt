cmake_minimum_required(VERSION 3.11)

project(TPRM_Examples)
set(CMAKE_BUILD_TYPE Release)

# Standalone examples (non-ROS)
add_executable(static_avoidance static_avoidance.cpp)
add_executable(dynamic_avoidance dynamic_avoidance.cpp)

target_link_libraries(static_avoidance t_prm)
target_link_libraries(dynamic_avoidance t_prm)

# ROS examples
if(CATKIN_DEVEL_PREFIX)
  # ROS node that integrates with fake_obs_sphere
  add_executable(ros_dynamic_test ros_dynamic_test.cpp)

  target_link_libraries(ros_dynamic_test
    t_prm
    ${catkin_LIBRARIES}
  )

  target_include_directories(ros_dynamic_test PRIVATE
    ${catkin_INCLUDE_DIRS}
  )

  # UTVD-TPRM ROS node
  add_executable(utvd_test UTVD_test.cpp)

  target_link_libraries(utvd_test
    t_prm
    ${catkin_LIBRARIES}
  )

  target_include_directories(utvd_test PRIVATE
    ${catkin_INCLUDE_DIRS}
  )

  # Install the ROS nodes
  install(TARGETS ros_dynamic_test utvd_test
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
