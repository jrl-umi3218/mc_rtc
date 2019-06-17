#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Try to find ROS and some required ROS packages
#
# If everything if found:
# - ROSCPP_FOUND is true
# - DEFINE_HAS_ROS_MACRO is "#define MC_RTC_HAS_ROS 1"
# - you can link with mc_rtc_3rd_party::ROS
#

pkg_check_modules(roscpp QUIET roscpp)
if(${roscpp_FOUND})
  set(ROSCPP_FOUND True)
  set(ROS_DEPENDENCIES roscpp;nav_msgs;sensor_msgs;tf2_ros;rosbag;mc_rtc_msgs)
  foreach(DEP ${ROS_DEPENDENCIES})
    pkg_check_modules(${DEP} REQUIRED ${DEP})
    list(APPEND ROS_LIBRARIES ${${DEP}_LIBRARIES})
    list(APPEND ROS_LIBRARY_DIRS ${${DEP}_LIBRARY_DIRS})
    list(APPEND ROS_INCLUDE_DIRS ${${DEP}_INCLUDE_DIRS})
    foreach(FLAG ${${DEP}_LDFLAGS})
      if(IS_ABSOLUTE ${FLAG})
        list(APPEND ROS_FULL_LIBRARIES ${FLAG})
      endif()
    endforeach()
  endforeach()
  list(REMOVE_DUPLICATES ROS_LIBRARIES)
  list(REMOVE_DUPLICATES ROS_LIBRARY_DIRS)
  list(REMOVE_DUPLICATES ROS_INCLUDE_DIRS)
  foreach(LIB ${ROS_LIBRARIES})
    string(SUBSTRING "${LIB}" 0 1 LIB_STARTS_WITH_COLUMN)
    if("${LIB_STARTS_WITH_COLUMN}" STREQUAL ":")
      string(SUBSTRING "${LIB}" 1 -1 LIB)
    endif()
    if(IS_ABSOLUTE ${LIB})
      list(APPEND ROS_FULL_LIBRARIES ${LIB})
    else()
      find_library(${LIB}_FULL_PATH NAME ${LIB} HINTS ${ROS_LIBRARY_DIRS})
      list(APPEND ROS_FULL_LIBRARIES ${${LIB}_FULL_PATH})
    endif()
  endforeach()
  list(REMOVE_DUPLICATES ROS_FULL_LIBRARIES)
  set(DEFINE_HAS_ROS_MACRO "#define MC_RTC_HAS_ROS 1")
  add_library(mc_rtc_3rd_party::ROS INTERFACE IMPORTED)
  set_target_properties(mc_rtc_3rd_party::ROS PROPERTIES
    INTERFACE_LINK_LIBRARIES "${ROS_FULL_LIBRARIES}"
    INTERFACE_INCLUDE_DIRECTORIES "${ROS_INCLUDE_DIRS}"
  )
  message("-- Found ROS libraries: ${ROS_FULL_LIBRARIES}")
  message("-- Found ROS include directories: ${ROS_INCLUDE_DIRS}")
else()
  set(ROSCPP_FOUND False)
  set(DEFINE_HAS_ROS_MACRO "")
endif()
