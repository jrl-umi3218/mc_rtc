#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

macro(robot_test_common NAME)
  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC mc_rbdyn)
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS"
    PREFIX ""
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    )
  set(TEST_ROBOT_NAME ${NAME})
  if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(TEST_ROBOT_NAME "${TEST_ROBOT_NAME}/$<CONFIGURATION>")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${NAME})
endmacro()

macro(robot_test_construction_failure NAME)
  robot_test_common(${NAME})
  set_target_properties(${NAME} PROPERTIES FOLDER tests/robots/failure)
  if(CMAKE_CONFIGURATION_TYPES)
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIGURATION>/mc_rtc-${NAME}.conf")
    file(GENERATE
       OUTPUT ${CONFIG_OUT}
       INPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  else()
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf")
  endif()
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=CONSTRUCTION_FAILURE -- ${CONFIG_OUT})
endmacro()

robot_test_construction_failure(NotARobot)
robot_test_construction_failure(NoDestroyRobot)
robot_test_construction_failure(NoCreateRobot)
robot_test_construction_failure(ThrowingRobot)
