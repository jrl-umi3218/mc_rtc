set(LOG_ENABLED "false")
set(LOG_POLICY "non-threaded")

macro(CONTROLLER_TEST_COMMON NAME)
  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} ${Boost_LIBRARIES} mc_control)
  pkg_config_use_dependency(${NAME} jsoncpp)
  pkg_config_use_dependency(${NAME} Tasks)
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    )
  set(TEST_CONTROLLER_NAME ${NAME})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  # Adding a project configuration file (for MSVC only)
  GENERATE_MSVC_DOT_USER_FILE(${NAME})
endmacro()

macro(CONTROLLER_TEST_CONSTRUCTION_FAILURE NAME)
  CONTROLLER_TEST_COMMON(${NAME})
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=CONSTRUCTION_FAILURE ${NAME}/mc_rtc-${NAME}.conf)
endmacro()

macro(CONTROLLER_TEST_RUN NAME NRITER)
  CONTROLLER_TEST_COMMON(${NAME})
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=RUN ${NAME}/mc_rtc-${NAME}.conf ${NRITER})
endmacro()

CONTROLLER_TEST_CONSTRUCTION_FAILURE(NotAController)
CONTROLLER_TEST_CONSTRUCTION_FAILURE(NoDestroyController)
CONTROLLER_TEST_CONSTRUCTION_FAILURE(NoCreateController)
# To run properly these tests require controller creation sandboxing
# which is only enabled on Linux
if(UNIX AND NOT APPLE)
  CONTROLLER_TEST_CONSTRUCTION_FAILURE(UnresolvedSymbolController)
  CONTROLLER_TEST_CONSTRUCTION_FAILURE(SegfaultController)
  CONTROLLER_TEST_CONSTRUCTION_FAILURE(FPEController)
endif()
CONTROLLER_TEST_CONSTRUCTION_FAILURE(ThrowingController)

set(LOG_ENABLED "true")
CONTROLLER_TEST_RUN(TestPostureController 400)
# These tests require a /tmp LogDirectory and symlinks to access the
# *-latest.log symlink created by mc_rtc
if(UNIX)
  add_test(TestPostureControllerLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController" 400)

  # Test threaded log policy
  set(LOG_POLICY "threaded")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/mc_rtc-TestPostureController-threaded-log.conf)
  add_test(NAME TestPostureControllerThreadedLogging COMMAND test_controller_ticker --run_test=RUN TestPostureController/mc_rtc-TestPostureController-threaded-log.conf 1200)
  add_test(TestPostureControllerThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController" 1200)

  # Test controller switch and threaded log policy
  add_library(TestPostureController2 SHARED TestPostureController2.cpp)
  target_link_libraries(TestPostureController2 ${Boost_LIBRARIES} mc_control)
  set_target_properties(TestPostureController2 PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController
    )
  set(TEST_CONTROLLER_NAME "TestPostureController\", \"TestPostureController2")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/mc_rtc-TestPostureController2.conf)
  add_test(NAME TestPostureController2 COMMAND test_controller_ticker --run_test=RUN TestPostureController/mc_rtc-TestPostureController2.conf 1200 TestPostureController2)
  add_test(TestPostureControllerControllerSwitchThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController" 1200)
  add_test(TestPostureController2ControllerSwitchThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController2" 1200)
endif()

set(LOG_POLICY "non-threaded")
# mc_task test controllers
CONTROLLER_TEST_RUN(TestCoMTaskController 4001)
CONTROLLER_TEST_RUN(TestPositionTaskController 5000)
CONTROLLER_TEST_RUN(TestOrientationTaskController 6001)
CONTROLLER_TEST_RUN(TestEndEffectorTaskController 4001)
