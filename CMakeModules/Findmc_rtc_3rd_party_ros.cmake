#
# Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find ROS and some required ROS packages
#
# If everything if found: mc_rtc_3rd_party::ROS exists
#

function(mc_rtc_ros2_dependency PKG TARGET)
  find_package(${PKG} REQUIRED)
  target_link_libraries(mc_rtc_3rd_party::ROS INTERFACE ${PKG}::${TARGET})
endfunction()

# Set ROS_VERSION to 2 if not provided
# NOTE: Kept for backwards compatibility with cmake script that expect it,
# but mc_rtc has dropped ROS1 support
if(NOT DEFINED ENV{ROS_VERSION} OR "$ENV{ROS_VERSION}" STREQUAL "")
  set(ROS_VERSION "2")
else()
  set(ROS_VERSION "$ENV{ROS_VERSION}")
endif()
if(ROS_VERSION STREQUAL "1")
  message(
    FATAL_ERROR "Since mc_rtc 2.15, ROS1 is not supported anymore, please use ROS2"
  )
endif()

if(NOT TARGET mc_rtc_3rd_party::ROS)
  if(NOT COMMAND pkg_check_modules)
    find_package(PkgConfig)
  endif()
  cmake_minimum_required(VERSION 3.22)
  list(APPEND CMAKE_PREFIX_PATH $ENV{AMENT_PREFIX_PATH})
  set(AMENT_CMAKE_UNINSTALL_TARGET
      OFF
      CACHE BOOL "" FORCE
  )
  # sets rclcpp_FOUND and rclcpp::rclcpp target if found
  find_package(rclcpp QUIET)
  if(NOT TARGET rclcpp::rclcpp)
    return()
  endif()
  add_library(mc_rtc_3rd_party::ROS INTERFACE IMPORTED)
  target_link_libraries(mc_rtc_3rd_party::ROS INTERFACE rclcpp::rclcpp)
  mc_rtc_ros2_dependency(nav_msgs nav_msgs__rosidl_typesupport_cpp)
  mc_rtc_ros2_dependency(sensor_msgs sensor_msgs__rosidl_typesupport_cpp)
  mc_rtc_ros2_dependency(mc_rtc_msgs mc_rtc_msgs__rosidl_typesupport_cpp)
  mc_rtc_ros2_dependency(tf2_ros tf2_ros)
  mc_rtc_ros2_dependency(rosbag2_cpp rosbag2_cpp)
  target_compile_definitions(
    mc_rtc_3rd_party::ROS INTERFACE MC_RTC_HAS_ROS_SUPPORT MC_RTC_ROS_IS_ROS2
  )
  # Legacy, we should use rclcpp_FOUND instead
  set(ROSCPP_FOUND True)
endif()
