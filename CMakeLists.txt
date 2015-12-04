#FIXME very crude initial cmake

cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)
include(cmake/eigen.cmake)
include(cmake/pkg-config.cmake)

set(PROJECT_NAME mc_rtc)
set(PROJECT_DESCRIPTION mc_rtc)
set(PROJECT_URL https://rob-dev.lirmm.fr/multi-contact/mc_rtc)

set(CXX_DISABLE_WERROR 1)

set(DOXYGEN_USE_MATHJAX "YES")
setup_project()

add_definitions(-std=c++0x)
add_definitions(-mfpmath=sse -msse2)
#add_definitions(-pedantic -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
#add_definitions(-Wno-sign-conversion -Wno-shorten-64-to-32)
#add_definitions(-Werror)

search_for_eigen()

find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

add_required_dependency(jsoncpp)
add_required_dependency(Tasks)
add_required_dependency(mc_rbdyn_urdf_cpp)

add_optional_dependency(hrpsys-base)
if(${HRPSYS_BASE_FOUND})
  set(DEFINE_HAS_HRPSYS_BASE_MACRO "#define MC_RTC_HAS_HRPSYS_BASE 1")
  include(cmake/idl.cmake)
  omniidl_include_directories("idl")
  add_subdirectory(idl)
  set_source_files_properties(${CMAKE_BINARY_DIR}/idl/MCControlService_idl.py PROPERTIES GENERATED 1)
  install(FILES ${CMAKE_BINARY_DIR}/idl/MCControlService_idl.py DESTINATION lib/python2.7/dist-packages/hrpsys)
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
      MESSAGE(WARNING "GEOS_PREFIX not defined, assuming it is equal to CMAKE_INSTALL_PREFIX")
      set(GEOS_PREFIX ${CMAKE_INSTALL_PREFIX})
    endif()
    set(GEOS_INCLUDE_DIR ${GEOS_PREFIX}/include)
    set(GEOS_LIBRARY_DIR ${GEOS_PREFIX}/lib)
    set(GEOS_LIBRARY geos)
    include_directories(${GEOS_INCLUDE_DIR})
    link_directories(${GEOS_LIBRARY_DIR})
  else()
    MESSAGE(SEND_ERROR "Could not find geos-config, missing dependency")
  endif()
endif()

if(NOT DEFINED MC_ENV_DESCRIPTION_PATH)
  execute_process(COMMAND rospack find mc_env_description
                  RESULT_VARIABLE MC_ENV_DESCRIPTION_FOUND
                  OUTPUT_VARIABLE MC_ENV_DESCRIPTION_PATH
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(${MC_ENV_DESCRIPTION_FOUND})
    MESSAGE(SEND_ERROR "Failed to find mc_env_description, check your system configuration or define MC_ENV_DESCRIPTION_PATH")
  else()
    MESSAGE(STATUS "Found mc_env_description: ${MC_ENV_DESCRIPTION_PATH}")
  endif()
endif()

if(NOT DEFINED HRP2_DRC_DESCRIPTION_PATH)
  execute_process(COMMAND rospack find hrp2_drc_description
                  RESULT_VARIABLE HRP2_DRC_DESCRIPTION_FOUND
                  OUTPUT_VARIABLE HRP2_DRC_DESCRIPTION_PATH
                  OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(${HRP2_DRC_DESCRIPTION_FOUND})
    MESSAGE(SEND_ERROR "Failed to find hrp2_drc_description, check your system configuration or define HRP2_DRC_DESCRIPTION_PATH")
  else()
    MESSAGE(STATUS "Found hrp2_drc_description: ${HRP2_DRC_DESCRIPTION_PATH}")
  endif()
endif()

add_optional_dependency(roscpp)
if(${ROSCPP_FOUND})
  set(DEFINE_HAS_ROS_MACRO "#define MC_RTC_HAS_ROS 1")
  add_required_dependency(sensor_msgs)
  add_required_dependency(tf2_ros)
else()
  set(DEFINE_HAS_ROS_MACRO "")
endif()

configure_file(${PROJECT_SOURCE_DIR}/include/mc_rtc/config.h.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/include/mc_rtc/config.h)

add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(utils)

install(DIRECTORY include ${CMAKE_BINARY_DIR}/include
          DESTINATION ${CMAKE_INSTALL_PREFIX}
          FILES_MATCHING REGEX "h(pp)?$")

install(
  FILES
    data/drc_stairs_climbing.json
    data/drc_stairs_climbing_config.json
    data/jvrc_ladder_climbing.json
    data/jvrc_ladder_climbing_config.json
    data/irex_ladder_climbing.json
    data/irex_ladder_climbing_config.json
    data/drc_car_egress.json
    data/drc_car_egress_config.json
  DESTINATION share/mc_rtc
)
install(
  FILES
    etc/mc_rtc.conf
  DESTINATION etc
)

setup_project_finalize()
