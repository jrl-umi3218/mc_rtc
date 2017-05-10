#FIXME very crude initial cmake

cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)
include(cmake/eigen.cmake)
include(cmake/pkg-config.cmake)
include(cmake/msvc-specific.cmake)

set(PROJECT_NAME mc_rtc)
set(PROJECT_DESCRIPTION mc_rtc)
set(PROJECT_URL https://rob-dev.lirmm.fr/multi-contact/mc_rtc)
set(PROJECT_DEBUG_POSTFIX "_d")

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

set(DOXYGEN_USE_MATHJAX "YES")
setup_project()

add_definitions(-std=c++0x)
add_definitions(-mfpmath=sse -msse2)
if(WIN32)
add_definitions("-DNOMINMAX")
endif()

SEARCH_FOR_EIGEN()

if(WIN32)
  # TODO: ask explicitly the user for the value of PKG_CONFIG_INCLUDE_DIR
  set( ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH};${PKG_CONFIG_INCLUDE_DIR}" )
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time timer regex)

add_required_dependency("Tasks" "Tasks_d")
add_required_dependency("mc_rbdyn_urdf_cpp" "mc_rbdyn_urdf_cpp_d")

# At this point, this dependency is used to get access to PD gains setting on HRP robots
add_optional_dependency(hrpsys-base)
if(${HRPSYS_BASE_FOUND})
  include_directories(${HRPSYS_BASE_INCLUDE_DIRS})
  link_directories(${HRPSYS_BASE_LIBRARY_DIRS})
  set(DEFINE_HAS_HRPSYS_BASE_MACRO "#define MC_RTC_HAS_HRPSYS_BASE 1")
else()
  set(DEFINE_HAS_HRPSYS_BASE_MACRO "")
endif()

# Find GEOS flags using geos-config
find_program(GEOS_CONFIG geos-config
  /usr/local/bin
  /usr/bin
  ${CMAKE_INSTALL_PREFIX}/bin
)
if(GEOS_CONFIG)
  exec_program(${GEOS_CONFIG}
    ARGS --prefix
    OUTPUT_VARIABLE GEOS_PREFIX)
  set(GEOS_INCLUDE_DIR ${GEOS_PREFIX}/include)
  set(GEOS_LIBRARY_DIR ${GEOS_PREFIX}/lib)
  set(GEOS_LIBRARY geos)
  include_directories(${GEOS_INCLUDE_DIR})
  link_directories(${GEOS_LIBRARY_DIR})
else()
  if(WIN32)
    if(NOT DEFINED GEOS_PREFIX)
      message(WARNING "GEOS_PREFIX not defined, assuming it is equal to CMAKE_INSTALL_PREFIX")
      set(GEOS_PREFIX ${CMAKE_INSTALL_PREFIX})
    endif()
    set(GEOS_INCLUDE_DIR ${GEOS_PREFIX}/include)
    set(GEOS_LIBRARY_DIR ${GEOS_PREFIX}/lib)
    set(GEOS_LIBRARY geos)
    include_directories(${GEOS_INCLUDE_DIR})
    link_directories(${GEOS_LIBRARY_DIR})
  else()
    message(SEND_ERROR "Could not find geos-config, missing dependency")
  endif()
endif()

macro(find_description_package desc_pkg)
  string(TOUPPER "${desc_pkg}" "DESC_PKG")
  if(NOT DEFINED ${DESC_PKG}_PATH)
    add_optional_dependency("${desc_pkg}")
    if(${${DESC_PKG}_FOUND})
      execute_process(
        COMMAND "${PKG_CONFIG_EXECUTABLE}"
        "--variable=datadir" "${desc_pkg}"
        OUTPUT_VARIABLE "${DESC_PKG}_PATH"
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    else()
      message(WARNING "${DESC_PKG}_PATH is not defined and pkg-config failed to find ${desc_pkg}, the related robots' modules will not work properly. Check your system configuration or define ${DESC_PKG} to fix this issue.")
    endif()
  endif()
  if(DEFINED ${DESC_PKG}_PATH)
    message(STATUS "Found ${desc_pkg}: ${${DESC_PKG}_PATH}")
  endif()
endmacro()

find_description_package(mc_env_description)
find_description_package(hrp2_drc_description)
find_description_package(hrp4_description)

add_optional_dependency(roscpp)
if(${ROSCPP_FOUND})
  set(DEFINE_HAS_ROS_MACRO "#define MC_RTC_HAS_ROS 1")
  add_required_dependency(nav_msgs)
  add_required_dependency(sensor_msgs)
  add_required_dependency(tf2_ros)
else()
  set(DEFINE_HAS_ROS_MACRO "")
endif()

option(CHOOSE_HRP4 "Use HRP4 as main robot instead of HRP2DRC" False)
if(CHOOSE_HRP4)
  set(DEFINE_CHOOSE_HRP4 "#define CHOOSE_HRP4 1")
else()
  set(DEFINE_CHOOSE_HRP4 "")
endif()

set(MC_ROBOTS_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/lib/mc_robots)
set(MC_CONTROLLER_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/lib/mc_controller)

configure_file(${PROJECT_SOURCE_DIR}/include/mc_rtc/config.h.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/include/mc_rtc/config.h)

# For MSVC, set local environment variable to enable finding the built dll
# of the main library when launching ctest with RUN_TESTS
if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif(MSVC)

if(WIN32)
  include_directories(3rd-party/ltdl)
endif()

###################
# -- RapidJSON -- #
###################

# The code was copied from the 1.1.0 release tarball for RapidJSON
# https://github.com/miloyip/rapidjson
include_directories(3rd-party/RapidJSON)

add_subdirectory(3rd-party)
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(utils)

install(DIRECTORY include ${CMAKE_BINARY_DIR}/include
          DESTINATION ${CMAKE_INSTALL_PREFIX}
          FILES_MATCHING REGEX "h(pp)?$")

file(GLOB data_files "data/*.json")
install(
  FILES
    ${data_files}
  DESTINATION share/mc_rtc
)
install(
  FILES
    etc/mc_rtc.conf
  DESTINATION etc
)
install(
  FILES
    etc/Seq.conf
  DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/etc/)

setup_project_finalize()
