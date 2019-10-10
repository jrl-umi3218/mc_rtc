#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Findhpp-spline.cmake
#
# Finds the hpp-spline library
# https://github.com/humanoid-path-planner/hpp-spline
#
# This will define the following imported targets
#
#     mc_rtc_3rd_party::hpp-spline

find_package(PkgConfig)
pkg_check_modules(hpp-spline QUIET hpp-spline)

if(hpp-spline_FOUND AND NOT TARGET hpp-spline::hpp-spline)
    add_library(mc_rtc_3rd_party::hpp-spline INTERFACE IMPORTED)
    set_target_properties(mc_rtc_3rd_party::hpp-spline PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${hpp-spline_INCLUDE_DIR}"
    )
endif()

