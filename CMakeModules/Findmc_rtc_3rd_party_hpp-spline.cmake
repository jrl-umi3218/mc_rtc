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

find_package(PkgConfig)
pkg_check_modules(hpp-spline QUIET hpp-spline)

if(${hpp-spline_FOUND})
  set(hpp-spline_FOUND True)
  if(hpp-spline_FOUND AND NOT TARGET hpp-spline::hpp-spline)
      add_library(mc_rtc_3rd_party::hpp-spline INTERFACE IMPORTED)
      set_target_properties(mc_rtc_3rd_party::hpp-spline PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${hpp-spline_INCLUDE_DIRS}"
      )
  endif()
  message("-- Found hpp-spline include directories: ${hpp-spline_INCLUDE_DIRS}")
else()
  set(hpp-spline_FOUND False)
endif()


