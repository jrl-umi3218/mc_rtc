macro(ROBOT_TEST_COMMON NAME)
  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} ${Boost_LIBRARIES} mc_rbdyn)
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS"
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    )
  set(TEST_ROBOT_NAME ${NAME})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
endmacro()

macro(ROBOT_TEST_CONSTRUCTION_FAILURE NAME)
  ROBOT_TEST_COMMON(${NAME})
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=CONSTRUCTION_FAILURE ${NAME}/mc_rtc-${NAME}.conf)
endmacro()

ROBOT_TEST_CONSTRUCTION_FAILURE(NotARobot)
ROBOT_TEST_CONSTRUCTION_FAILURE(NoDestroyRobot)
ROBOT_TEST_CONSTRUCTION_FAILURE(NoCreateRobot)
# To run properly this test requires controller creation sandboxing
# which is only enabled on Linux
if(UNIX AND NOT APPLE)
  ROBOT_TEST_CONSTRUCTION_FAILURE(UnresolvedSymbolRobot)
  ROBOT_TEST_CONSTRUCTION_FAILURE(SegfaultRobot)
  ROBOT_TEST_CONSTRUCTION_FAILURE(FPERobot)
endif()
ROBOT_TEST_CONSTRUCTION_FAILURE(ThrowingRobot)
