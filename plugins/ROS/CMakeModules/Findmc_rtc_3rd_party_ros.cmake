#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find ROS and some required ROS packages
#
# If everything if found:
# - ROSCPP_FOUND is true
# - you can link with mc_rtc_3rd_party::ROS
#

if(NOT TARGET mc_rtc_3rd_party::ROS)
  include(FindPkgConfig)
  pkg_check_modules(MC_RTC_roscpp QUIET roscpp)
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
    set_target_properties(mc_rtc_3rd_party::ROS PROPERTIES
      INTERFACE_LINK_LIBRARIES "${MC_RTC_ROS_FULL_LIBRARIES}"
      INTERFACE_INCLUDE_DIRECTORIES "${MC_RTC_ROS_INCLUDE_DIRS}"
    )
    message("-- Found ROS libraries: ${MC_RTC_ROS_FULL_LIBRARIES}")
    message("-- Found ROS include directories: ${MC_RTC_ROS_INCLUDE_DIRS}")
  else()
    set(ROSCPP_FOUND False)
  endif()
endif()
