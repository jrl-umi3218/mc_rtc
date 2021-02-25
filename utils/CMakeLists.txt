#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

if(NOT DEFINED MC_LOG_UI_PYTHON_EXECUTABLE)
  set(MC_LOG_UI_PYTHON_EXECUTABLE "python")
endif()

find_package(Boost REQUIRED COMPONENTS program_options timer)

macro(add_mc_rtc_utils NAME)
  add_executable(${NAME} ${ARGN})
  set_target_properties(${NAME} PROPERTIES FOLDER utils)
  target_link_libraries(${NAME} PUBLIC mc_control)
  install(TARGETS ${NAME} DESTINATION bin)
endmacro()

add_mc_rtc_utils(mc_bin_to_log mc_bin_to_log.cpp mc_bin_to_log_main.cpp)

add_mc_rtc_utils(mc_bin_to_flat mc_bin_to_flat.cpp mc_bin_to_flat_main.cpp)

add_mc_rtc_utils(mc_bin_perf mc_bin_perf.cpp)

add_mc_rtc_utils(mc_old_bin_to_flat mc_old_bin_to_flat.cpp)

add_mc_rtc_utils(mc_json_to_yaml mc_json_to_yaml.cpp)

configure_file(mc_bin_utils.in.cpp "${CMAKE_CURRENT_BINARY_DIR}/mc_bin_utils.cpp")
set(mc_bin_utils_SRC
  "${CMAKE_CURRENT_BINARY_DIR}/mc_bin_utils.cpp"
  mc_bin_to_log.cpp
  mc_bin_to_flat.cpp
)
add_mc_rtc_utils(mc_bin_utils ${mc_bin_utils_SRC})
target_include_directories(mc_bin_utils PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(mc_bin_utils PUBLIC Boost::program_options Boost::disable_autolinking)
if(NOT MC_RTC_BUILD_STATIC)
  target_link_libraries(mc_bin_utils PRIVATE mpack)
else()
endif()

configure_file(mc_rtc_new_controller.in.py "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_new_controller" @ONLY)
set(AROBASE "@")
configure_file(mc_rtc_new_fsm_controller.in.py "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_new_fsm_controller" @ONLY)
unset(AROBASE)
if(NOT WIN32)
  set(UTILS_SCRIPT
    "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_new_controller"
    "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_new_fsm_controller"
  )
  install(PROGRAMS ${UTILS_SCRIPT} DESTINATION bin)
else()
  set(UTILS_SCRIPT
    mc_rtc_new_controller
    mc_rtc_new_fsm_controller
  )
  foreach(PGM ${UTILS_SCRIPT})
    install(PROGRAMS "${CMAKE_CURRENT_BINARY_DIR}/${PGM}" DESTINATION bin RENAME ${PGM}.py)
  endforeach()
endif()

add_subdirectory(mc_log_gui)
