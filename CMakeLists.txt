#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1)
if(POLICY CMP0063)
  cmake_policy(SET CMP0063 NEW)
endif()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

include(cmake/base.cmake)
include(cmake/cython/cython.cmake)
include(cmake/msvc-specific.cmake)
include(cmake/version-script.cmake)

set(PROJECT_NAME mc_rtc)
set(PROJECT_DESCRIPTION mc_rtc)
set(PROJECT_URL https://gite.lirmm.fr/multi-contact/mc_rtc)
set(PROJECT_DEBUG_POSTFIX "_d")
set(PROJECT_USE_KEYWORD_LINK_LIBRARIES TRUE)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_STANDARD 11)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(DOXYGEN_USE_MATHJAX "YES")
setup_project()

option(GENERATE_COVERAGE "Generate coverage data" FALSE)
if("${CMAKE_CXX_FLAGS}" MATCHES "--coverage")
  set(GENERATE_COVERAGE TRUE CACHE BOOL "Generate coverage data" FORCE)
  string(REPLACE "--coverage" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  string(REPLACE "--coverage" "" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
  string(REPLACE "--coverage" "" CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS}")
endif()

if(WIN32)
  add_definitions("-DNOMINMAX" "-D_ENABLE_EXTENDED_ALIGNED_STORAGE")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4068")
else()
  add_definitions(-mfpmath=sse -msse2)
endif()

option(BUILD_CONTROLLER_SAMPLES "Build sample controllers in the project" ON)
option(DISABLE_TESTS "Disable unit tests" OFF)
option(DISABLE_CONTROLLER_TESTS "Disable controller unit tests" OFF)
option(DISABLE_ROBOT_TESTS "Disable RobotModule unit tests" OFF)

option(BUILD_BENCHMARKS "Build benchmarks" OFF)

option(DISABLE_ROS "Do not build with ROS support (even if ROS was found)" OFF)

add_project_dependency(Boost REQUIRED COMPONENTS filesystem timer)
add_project_dependency(Tasks REQUIRED)
add_project_dependency(mc_rbdyn_urdf REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

# LTDL (defines target: mc_rtc_3rd_party::ltdl)
add_subdirectory(3rd-party/ltdl)

# GEOS (defines/imports target: GEOS::geos)
find_package(mc_rtc_3rd_party_geos REQUIRED)

# nanomsg (defines/imports target: nanomsg)
find_package(mc_rtc_3rd_party_nanomsg REQUIRED)

# qhull (build re-entrant static version)
add_subdirectory(3rd-party/qhull)

# RapidJSON
# The code was copied from the 1.1.0 release tarball for RapidJSON
# https://github.com/miloyip/rapidjson
add_library(mc_rtc_3rd_party::RapidJSON INTERFACE IMPORTED)
set_target_properties(mc_rtc_3rd_party::RapidJSON PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/RapidJSON
)

# ROS (defines target: mc_rtc_3rd_party::ROS)
if(NOT ${DISABLE_ROS})
  find_package(mc_rtc_3rd_party_ros REQUIRED)
else()
  set(ROSCPP_FOUND False)
endif()

macro(find_description_package desc_pkg)
  string(TOUPPER "${desc_pkg}" "DESC_PKG")
  if(NOT DEFINED ${DESC_PKG}_PATH)
    execute_process(
      COMMAND "${PKG_CONFIG_EXECUTABLE}"
      "--variable=datadir" "${desc_pkg}"
      RESULT_VARIABLE "${DESC_PKG}_RESULT"
      OUTPUT_VARIABLE "${DESC_PKG}_PATH"
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(${${DESC_PKG}_RESULT})
      set(${DESC_PKG}_FOUND False)
      message(WARNING "${DESC_PKG}_PATH is not defined and pkg-config failed to find ${desc_pkg}, the related robots' modules will not work properly. Check your system configuration or define ${DESC_PKG} to fix this issue.")
    else()
      message(STATUS "${desc_pkg}: ${${DESC_PKG}_PATH}")
      set(${DESC_PKG}_FOUND True)
    endif()
  else()
    set(${DESC_PKG}_FOUND True)
    message(STATUS "${desc_pkg}: ${${DESC_PKG}_PATH}")
  endif()
endmacro()

find_description_package(mc_env_description)

set(MC_ROBOTS_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/lib/mc_robots)
set(MC_CONTROLLER_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/lib/mc_controller)

configure_file(${PROJECT_SOURCE_DIR}/include/mc_rtc/config.h.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/include/mc_rtc/config.h)

# For MSVC, set local environment variable to enable finding the built dll
# of the main library when launching ctest with RUN_TESTS
if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif(MSVC)

add_subdirectory(CMakeModules)
add_subdirectory(doc)
add_subdirectory(idl)
add_subdirectory(src)
if(NOT ${DISABLE_TESTS})
  add_subdirectory(tests)
endif()
if(${BUILD_BENCHMARKS})
  add_subdirectory(benchmarks)
endif()
add_subdirectory(utils)

if(${PYTHON_BINDING})
 add_subdirectory(binding/python)
endif()

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

setup_project_finalize()
setup_project_package_finalize()

# Extra macros for mc_rtc
install(FILES "src/mc_rtcMacros.cmake"
        DESTINATION "${CONFIG_INSTALL_DIR}")
file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/generated/mc_rtcConfig.cmake" "include(\"\${CMAKE_CURRENT_LIST_DIR}/mc_rtcMacros.cmake\")\n")
file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/generated/mc_rtcConfig.cmake" "include(\"\${CMAKE_CURRENT_LIST_DIR}/mc_rtc_fsm_states.cmake\")\n")
