#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Findhpp-spline.cmake
#
# Finds the hpp-spline library
# https://github.com/humanoid-path-planner/hpp-spline
#
# The following variables will be set
#
# hpp-spline_FOUND False
#
# If hpp-spline is found the following imported targets will be defined
#
#     mc_rtc_3rd_party::hpp-spline

if(NOT TARGET mc_rtc_3rd_party::hpp-spline)
find_package(PkgConfig)
pkg_check_modules(hpp-spline QUIET hpp-spline)

# Look for the header if pkg-config failed for some reason
if(NOT hpp-spline_FOUND)
  if(NOT DEFINED hpp-spline_PREFIX)
    set(hpp-spline_PREFIX "${CMAKE_INSTALL_PREFIX}")
  endif()
  find_path(hpp-spline_INCLUDE_DIRS
    NAMES hpp/spline/exact_cubic.h
    HINTS ${hpp-spline_PREFIX})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(hpp-spline DEFAULT_MSG hpp-spline_INCLUDE_DIRS)
  mark_as_advanced(hpp-spline_INCLUDE_DIRS)
endif()

if(hpp-spline_FOUND)
  if(hpp-spline_FOUND AND NOT TARGET hpp-spline::hpp-spline)
      add_library(mc_rtc_3rd_party::hpp-spline INTERFACE IMPORTED)
      set_target_properties(mc_rtc_3rd_party::hpp-spline PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${hpp-spline_INCLUDE_DIRS}"
      )
  endif()
  message("-- Found hpp-spline include directories: ${hpp-spline_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "Could not find hpp-spline")
endif()
endif()
