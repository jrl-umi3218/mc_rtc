set(LOG_ENABLED "false")
set(LOG_POLICY "non-threaded")

macro(controller_test_common NAME)
  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} ${Boost_LIBRARIES} mc_control)
  pkg_config_use_dependency(${NAME} Tasks)
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    )
  set(TEST_CONTROLLER_NAME ${NAME})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${NAME})
endmacro()

macro(controller_test_construction_failure NAME)
  controller_test_common(${NAME})
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=CONSTRUCTION_FAILURE ${NAME}/mc_rtc-${NAME}.conf)
endmacro()

macro(controller_test_run NAME NRITER)
  controller_test_common(${NAME})
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=RUN ${NAME}/mc_rtc-${NAME}.conf ${NRITER})
endmacro()

controller_test_construction_failure(NotAController)
controller_test_construction_failure(NoDestroyController)
controller_test_construction_failure(NoCreateController)
# To run properly these tests require controller creation sandboxing
# which is only enabled on Linux
if(UNIX AND NOT APPLE)
  controller_test_construction_failure(UnresolvedSymbolController)
  controller_test_construction_failure(SegfaultController)
  controller_test_construction_failure(FPEController)
endif()
controller_test_construction_failure(ThrowingController)

set(LOG_ENABLED "true")
controller_test_run(TestPostureController 400)
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
controller_test_run(TestCoMTaskController 4001)
controller_test_run(TestPositionTaskController 5000)
controller_test_run(TestOrientationTaskController 6001)
controller_test_run(TestEndEffectorTaskController 4001)

# Test Python bindings
if(NOT DEFINED PYTHON_DEB_ROOT AND ${PYTHON_BINDING})
  set(TEST_CONTROLLER_NAME "PythonController#test_python.TestPythonController")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/TestPythonController/mc_rtc-TestPythonController.conf)
  add_test(NAME "TestPythonController" COMMAND test_controller_ticker --run_test=RUN TestPythonController/mc_rtc-TestPythonController.conf 400 "" "${CMAKE_CURRENT_SOURCE_DIR}/test_python")
  add_test(TestPythonControllerLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "${TEST_CONTROLLER_NAME}" 400)
endif()
