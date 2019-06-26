#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

macro(robot_test_common NAME)
  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC mc_rbdyn)
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS"
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    )
  set(TEST_ROBOT_NAME ${NAME})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${NAME})
endmacro()

macro(robot_test_construction_failure NAME)
  robot_test_common(${NAME})
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=CONSTRUCTION_FAILURE ${NAME}/mc_rtc-${NAME}.conf)
endmacro()

robot_test_construction_failure(NotARobot)
robot_test_construction_failure(NoDestroyRobot)
robot_test_construction_failure(NoCreateRobot)
# To run properly this test requires controller creation sandboxing
# which is only enabled on Linux
if(UNIX AND NOT APPLE)
  robot_test_construction_failure(UnresolvedSymbolRobot)
  robot_test_construction_failure(SegfaultRobot)
  if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    robot_test_construction_failure(FPERobot)
  endif()
endif()
robot_test_construction_failure(ThrowingRobot)
