#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Findndcurves.cmake
#
# Finds the ndcurves library
# https://github.com/humanoid-path-planner/ndcurves
#
# The following variables will be set
#
# ndcurves_FOUND False
#
# If ndcurves is found the following imported targets will be defined
#
#     mc_rtc_3rd_party::ndcurves

if(NOT TARGET mc_rtc_3rd_party::ndcurves)
find_package(PkgConfig)
pkg_check_modules(ndcurves QUIET ndcurves)

# Look for the header if pkg-config failed for some reason
if(NOT ndcurves_FOUND)
  if(NOT DEFINED ndcurves_PREFIX)
    set(ndcurves_PREFIX "${CMAKE_INSTALL_PREFIX}")
  endif()
  find_path(ndcurves_INCLUDE_DIRS
    NAMES hpp/spline/exact_cubic.h
    HINTS ${ndcurves_PREFIX})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(ndcurves DEFAULT_MSG ndcurves_INCLUDE_DIRS)
  mark_as_advanced(ndcurves_INCLUDE_DIRS)
endif()

if(ndcurves_FOUND)
  if(ndcurves_FOUND AND NOT TARGET ndcurves::ndcurves)
      add_library(mc_rtc_3rd_party::ndcurves INTERFACE IMPORTED)
      set_target_properties(mc_rtc_3rd_party::ndcurves PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ndcurves_INCLUDE_DIRS}"
      )
  endif()
  message("-- Found ndcurves include directories: ${ndcurves_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "Could not find ndcurves")
endif()
endif()
