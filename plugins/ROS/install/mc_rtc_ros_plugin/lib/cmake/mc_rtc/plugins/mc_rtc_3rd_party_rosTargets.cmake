#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find ROS and some required ROS packages
#
# If everything if found: - ROSCPP_FOUND is true - you can link with
# mc_rtc_3rd_party::ROS
#

function(mc_rtc_ros2_dependency PKG TARGET)
  find_package(${PKG} REQUIRED)
  target_link_libraries(mc_rtc_3rd_party::ROS INTERFACE ${PKG}::${TARGET})
endfunction()

if(NOT TARGET mc_rtc_3rd_party::ROS)
  if(NOT COMMAND pkg_check_modules)
    find_package(PkgConfig)
  endif()
  if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" EQUAL "2")
    cmake_minimum_required(VERSION 3.22)
    list(APPEND CMAKE_PREFIX_PATH $ENV{AMENT_PREFIX_PATH})
    set(AMENT_CMAKE_UNINSTALL_TARGET
        OFF
        CACHE BOOL "" FORCE
    )
    find_package(rclcpp QUIET)
    if(NOT TARGET rclcpp::rclcpp)
      set(ROSCPP_FOUND False)
      return()
    endif()
    add_library(mc_rtc_3rd_party::ROS INTERFACE IMPORTED)
    target_link_libraries(mc_rtc_3rd_party::ROS INTERFACE rclcpp::rclcpp)
    mc_rtc_ros2_dependency(nav_msgs nav_msgs__rosidl_typesupport_cpp)
    mc_rtc_ros2_dependency(sensor_msgs sensor_msgs__rosidl_typesupport_cpp)
    mc_rtc_ros2_dependency(mc_rtc_msgs mc_rtc_msgs__rosidl_typesupport_cpp)
    mc_rtc_ros2_dependency(tf2_ros tf2_ros)
    mc_rtc_ros2_dependency(rosbag2_cpp rosbag2_cpp)
    target_compile_definitions(mc_rtc_3rd_party::ROS INTERFACE MC_RTC_ROS_IS_ROS2)
    set(ROSCPP_FOUND True)
    return()
  else()
    pkg_check_modules(MC_RTC_roscpp QUIET roscpp)
  endif()
  if(${MC_RTC_roscpp_FOUND})
    set(ROSCPP_FOUND True)
    set(MC_RTC_ROS_DEPENDENCIES roscpp;nav_msgs;sensor_msgs;tf2_ros;rosbag;mc_rtc_msgs)
    foreach(DEP ${MC_RTC_ROS_DEPENDENCIES})
      pkg_check_modules(MC_RTC_${DEP} REQUIRED ${DEP})
      list(APPEND MC_RTC_ROS_LIBRARIES ${MC_RTC_${DEP}_LIBRARIES})
      list(APPEND MC_RTC_ROS_LIBRARY_DIRS ${MC_RTC_${DEP}_LIBRARY_DIRS})
      list(APPEND MC_RTC_ROS_INCLUDE_DIRS ${MC_RTC_${DEP}_INCLUDE_DIRS})
      foreach(FLAG ${MC_RTC_${DEP}_LDFLAGS})
        if(IS_ABSOLUTE ${FLAG})
          list(APPEND MC_RTC_ROS_FULL_LIBRARIES ${FLAG})
        endif()
      endforeach()
    endforeach()
    list(REMOVE_DUPLICATES MC_RTC_ROS_LIBRARIES)
    list(REMOVE_DUPLICATES MC_RTC_ROS_LIBRARY_DIRS)
    list(REMOVE_DUPLICATES MC_RTC_ROS_INCLUDE_DIRS)
    foreach(LIB ${MC_RTC_ROS_LIBRARIES})
      string(SUBSTRING "${LIB}" 0 1 LIB_STARTS_WITH_COLUMN)
      if(${LIB_STARTS_WITH_COLUMN} STREQUAL ":")
        string(SUBSTRING "${LIB}" 1 -1 LIB)
      endif()
      if(IS_ABSOLUTE ${LIB})
        list(APPEND MC_RTC_ROS_FULL_LIBRARIES ${LIB})
      else()
        find_library(${LIB}_FULL_PATH NAME ${LIB} HINTS ${MC_RTC_ROS_LIBRARY_DIRS})
        list(APPEND MC_RTC_ROS_FULL_LIBRARIES ${${LIB}_FULL_PATH})
      endif()
    endforeach()
    list(REMOVE_DUPLICATES MC_RTC_ROS_FULL_LIBRARIES)
    add_library(mc_rtc_3rd_party::ROS INTERFACE IMPORTED)
    set_target_properties(
      mc_rtc_3rd_party::ROS
      PROPERTIES INTERFACE_LINK_LIBRARIES "${MC_RTC_ROS_FULL_LIBRARIES}"
                 INTERFACE_INCLUDE_DIRECTORIES "${MC_RTC_ROS_INCLUDE_DIRS}"
    )
    message("-- Found ROS libraries: ${MC_RTC_ROS_FULL_LIBRARIES}")
    message("-- Found ROS include directories: ${MC_RTC_ROS_INCLUDE_DIRS}")
  else()
    set(ROSCPP_FOUND False)
  endif()
endif()
