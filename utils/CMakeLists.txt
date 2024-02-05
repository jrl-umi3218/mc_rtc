#
# Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
#

if(NOT DEFINED MC_LOG_UI_PYTHON_EXECUTABLE)
  if(PYTHON_BINDING_FORCE_PYTHON3)
    set(MC_LOG_UI_PYTHON_EXECUTABLE "python3")
  elseif(PYTHON_BINDING_FORCE_PYTHON2)
    set(MC_LOG_UI_PYTHON_EXECUTABLE "python2")
  else()
    set(MC_LOG_UI_PYTHON_EXECUTABLE "python")
  endif()
endif()

find_package(Boost REQUIRED COMPONENTS program_options timer)

function(add_mc_rtc_utils NAME)
  set(SOURCES ${ARGN})
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cpp)
    list(APPEND SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cpp)
  endif()
  add_executable(${NAME} ${SOURCES})
  set_target_properties(${NAME} PROPERTIES FOLDER utils)
  target_link_libraries(${NAME} PUBLIC mc_control)
  install(TARGETS ${NAME} DESTINATION bin)
endfunction()

add_mc_rtc_utils(mc_bin_to_log mc_bin_to_log_main.cpp)

add_mc_rtc_utils(mc_bin_to_flat mc_bin_to_flat_main.cpp)

add_mc_rtc_utils(mc_bin_perf)

add_mc_rtc_utils(mc_old_bin_to_flat)

add_mc_rtc_utils(mc_json_to_yaml)

add_library(RobotVisualizer OBJECT RobotVisualizer.h RobotVisualizer.cpp)
target_link_libraries(RobotVisualizer PUBLIC mc_rtc::mc_control)
if(TARGET mc_rtc::mc_rtc_ros)
  target_link_libraries(RobotVisualizer PUBLIC mc_rtc::mc_rtc_ros)
  target_compile_definitions(RobotVisualizer PUBLIC MC_RTC_HAS_ROS)
endif()

add_mc_rtc_utils(mc_convex_visualization)
target_link_libraries(mc_convex_visualization PRIVATE RobotVisualizer)
add_mc_rtc_utils(mc_robot_visualization)
target_link_libraries(mc_robot_visualization PRIVATE RobotVisualizer)
add_mc_rtc_utils(mc_surfaces_visualization)
target_link_libraries(mc_surfaces_visualization PRIVATE RobotVisualizer)

add_mc_rtc_utils(mc_rtc_ticker)
target_link_libraries(
  mc_rtc_ticker PUBLIC Boost::program_options Boost::disable_autolinking
)

configure_file(mc_bin_utils.in.cpp "${CMAKE_CURRENT_BINARY_DIR}/mc_bin_utils.cpp")
set(mc_bin_utils_SRC "${CMAKE_CURRENT_BINARY_DIR}/mc_bin_utils.cpp" mc_bin_to_log.cpp
                     mc_bin_to_flat.cpp
)
add_mc_rtc_utils(mc_bin_utils ${mc_bin_utils_SRC})
target_include_directories(mc_bin_utils PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(
  mc_bin_utils PUBLIC Boost::program_options Boost::disable_autolinking
)
if(NOT Boost_USE_STATIC_LIBS)
  target_link_libraries(mc_bin_utils PUBLIC Boost::dynamic_linking)
endif()
if(NOT MC_RTC_BUILD_STATIC)
  target_link_libraries(mc_bin_utils PRIVATE mpack)
else()

endif()

find_program(PYTHON_PGM ${MC_LOG_UI_PYTHON_EXECUTABLE})
if(NOT PYTHON_PGM)
  return()
endif()

configure_file(
  mc_rtc_new_controller.in.py "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_new_controller" @ONLY
)
set(AROBASE "@")
configure_file(
  mc_rtc_new_fsm_controller.in.py
  "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_new_fsm_controller" @ONLY
)
unset(AROBASE)
if(NOT WIN32)
  set(UTILS_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_new_controller"
                   "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_new_fsm_controller"
  )
  install(PROGRAMS ${UTILS_SCRIPT} DESTINATION bin)
else()
  set(UTILS_SCRIPT mc_rtc_new_controller mc_rtc_new_fsm_controller)
  foreach(PGM ${UTILS_SCRIPT})
    install(
      PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${PGM}"
      DESTINATION bin
      RENAME ${PGM}.py
    )
  endforeach()
endif()

add_subdirectory(mc_log_gui)
