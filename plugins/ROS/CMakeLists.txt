cmake_minimum_required(VERSION 3.1)

if(NOT DEFINED PROJECT_VERSION)
  set(PROJECT_VERSION 1.0.0)
endif()
project(mc_rtc_ros_plugin LANGUAGES CXX VERSION ${PROJECT_VERSION})

if(POLICY CMP0063)
  cmake_policy(SET CMP0063 NEW)
endif()

# Detect if we are building inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
  set(CONFIG_INSTALL_DIR "lib/cmake/mc_rtc/")
else()
  if(DISABLE_ROS)
    # Stop right here
    return()
  endif()
endif()
set(TARGETS_EXPORT_NAME "mc_rtc_rosTargets")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
find_package(mc_rtc_3rd_party_ros REQUIRED)
if(NOT ${ROSCPP_FOUND})
  return()
endif()

set(mc_rtc_ros_SRC src/mc_rtc_ros/ros.cpp)
set(mc_rtc_ros_HDR include/mc_rtc/ros.h include/mc_rtc/ros_api.h)
add_library(mc_rtc_ros SHARED ${mc_rtc_ros_SRC} ${mc_rtc_ros_HDR})
set_target_properties(mc_rtc_ros PROPERTIES COMPILE_FLAGS "-DMC_RTC_ROS_EXPORTS")
target_link_libraries(mc_rtc_ros PUBLIC mc_rtc::mc_rbdyn mc_rtc_3rd_party::ROS)
target_include_directories(mc_rtc_ros PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
set_target_properties(mc_rtc_ros PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
install(FILES ${mc_rtc_ros_HDR} DESTINATION include/mc_rtc)

set(mc_tasks_ros_SRC
  src/mc_tasks/LookAtTFTask.cpp)
set(mc_tasks_ros_HDR
  include/mc_tasks/LookAtTFTask.h
  include/mc_tasks/ros_api.h)
add_library(mc_tasks_ros SHARED ${mc_tasks_ros_SRC} ${mc_tasks_ros_HDR})
set_target_properties(mc_rtc_ros PROPERTIES COMPILE_FLAGS "-DMC_TASKS_ROS_EXPORTS")
target_link_libraries(mc_tasks_ros PUBLIC mc_rtc_ros mc_rtc::mc_tasks)
set_target_properties(mc_tasks_ros PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR} VERSION ${PROJECT_VERSION})
install(FILES ${mc_tasks_ros_HDR} DESTINATION include/mc_tasks)

set(mc_bin_to_rosbag_SRC
  utils/mc_bin_to_rosbag.cpp
  utils/mc_bin_to_rosbag.h
  utils/mc_bin_to_rosbag_main.cpp)
add_executable(mc_bin_to_rosbag ${mc_bin_to_rosbag_SRC})
set_target_properties(mc_bin_to_rosbag PROPERTIES FOLDER utils)
target_link_libraries(mc_bin_to_rosbag PUBLIC mc_rtc::mc_control mc_rtc_ros)
install(TARGETS mc_bin_to_rosbag DESTINATION bin)

install(TARGETS mc_rtc_ros mc_tasks_ros
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(FILES "${PROJECT_SOURCE_DIR}/CMakeModules/Findmc_rtc_3rd_party_ros.cmake"
  DESTINATION "${CONFIG_INSTALL_DIR}/plugins/"
  RENAME "mc_rtc_3rd_party_rosTargets.cmake")

install(
  EXPORT "${TARGETS_EXPORT_NAME}"
  NAMESPACE "mc_rtc::"
  DESTINATION "${CONFIG_INSTALL_DIR}/plugins/"
)

set(plugin_SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/ROS.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/Services.cpp")
set(plugin_HDR
  "${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/ROS.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/plugin/Services.h")
add_plugin(ROS "${plugin_SRC}" "${plugin_HDR}")
set_target_properties(ROS PROPERTIES COMPILE_FLAGS "-DMC_RTC_ROS_PLUGIN_EXPORTS")
target_link_libraries(ROS PUBLIC mc_rtc_ros mc_tasks_ros)
install(FILES etc/ROS.yaml DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")
