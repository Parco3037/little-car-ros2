cmake_minimum_required(VERSION 3.16)
project(raicom_slam_heading_gate LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_SKIP_RPATH TRUE)
find_package(ament_cmake REQUIRED)
set(dependencies rclcpp message_filters nav_msgs sensor_msgs tf2 tf2_ros
  visualization_msgs pluginlib tf2_geometry_msgs tf2_sensor_msgs std_msgs
  std_srvs builtin_interfaces interactive_markers)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread)
find_package(Eigen3 REQUIRED)
set(ROS_PREFIX /opt/ros/humble)
if(NOT IS_DIRECTORY "${UPSTREAM_SOURCE}/src")
  message(FATAL_ERROR "UPSTREAM_SOURCE must point to the preserved exact 2.6.8 source")
endif()
add_library(toolbox_common SHARED
  patched/slam_toolbox_common.cpp
  ${UPSTREAM_SOURCE}/src/map_saver.cpp
  ${UPSTREAM_SOURCE}/src/loop_closure_assistant.cpp
  ${UPSTREAM_SOURCE}/src/laser_utils.cpp
  ${UPSTREAM_SOURCE}/src/slam_mapper.cpp)
ament_target_dependencies(toolbox_common ${dependencies})
target_include_directories(toolbox_common PRIVATE
  ${ROS_PREFIX}/include ${ROS_PREFIX}/include/slam_toolbox ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(toolbox_common ${ROS_PREFIX}/lib/libkartoSlamToolbox.so
  ${ROS_PREFIX}/lib/libslam_toolbox__rosidl_typesupport_cpp.so ${Boost_LIBRARIES})
target_link_options(toolbox_common PRIVATE -Wl,-z,defs)
install(TARGETS toolbox_common LIBRARY DESTINATION lib)

add_executable(gate_runtime_test gate_runtime_test.cpp)
ament_target_dependencies(gate_runtime_test ${dependencies})
target_include_directories(gate_runtime_test PRIVATE
  ${ROS_PREFIX}/include ${ROS_PREFIX}/include/slam_toolbox ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(gate_runtime_test toolbox_common
  ${ROS_PREFIX}/lib/libkartoSlamToolbox.so ${Boost_LIBRARIES})
