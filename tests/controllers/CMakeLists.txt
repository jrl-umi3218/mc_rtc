#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(LOG_ENABLED "false")
set(LOG_POLICY "non-threaded")
set(USE_SANDBOX "true")

macro(controller_test_common NAME)
  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} PUBLIC mc_control)
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

set(LOG_ENABLED "true")
controller_test_run(TestPostureController 400)
# These tests require a /tmp LogDirectory and symlinks to access the
# *-latest.log symlink created by mc_rtc
if(UNIX)
  set(UTILS_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../utils)
  add_test(TestPostureControllerLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController" 400 "${UTILS_BIN_DIR}")

  # Test threaded log policy
  set(LOG_POLICY "threaded")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/mc_rtc-TestPostureController-threaded-log.conf)
  add_test(NAME TestPostureControllerThreadedLogging COMMAND test_controller_ticker --run_test=RUN TestPostureController/mc_rtc-TestPostureController-threaded-log.conf 1500)
  add_test(TestPostureControllerThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController" 1500 "${UTILS_BIN_DIR}")

  # Test controller switch and threaded log policy
  add_library(TestPostureController2 SHARED TestPostureController2.cpp)
  target_link_libraries(TestPostureController2 PUBLIC ${Boost_LIBRARIES} mc_control)
  set_target_properties(TestPostureController2 PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController
    )
  set(TEST_CONTROLLER_NAME "TestPostureController\", \"TestPostureController2")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/mc_rtc-TestPostureController2.conf)
  add_test(NAME TestPostureController2 COMMAND test_controller_ticker --run_test=RUN TestPostureController/mc_rtc-TestPostureController2.conf 1200 TestPostureController2)
  add_test(TestPostureControllerControllerSwitchThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController" 1200 "${UTILS_BIN_DIR}")
  add_test(TestPostureController2ControllerSwitchThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController2" 1200 "${UTILS_BIN_DIR}")
endif()

set(LOG_POLICY "non-threaded")
# mc_task test controllers
controller_test_run(TestCoMTaskController 4001)
controller_test_run(TestPositionTaskController 5000)
controller_test_run(TestOrientationTaskController 6001)
controller_test_run(TestEndEffectorTaskController 4001)

# Test Python bindings
if(${PYTHON_BINDING})
  set(USE_SANDBOX "false")
  get_python_names(python_names)
  foreach(PYTHON ${python_names})
    set(TEST_CONTROLLER_NAME "${PYTHON}#test_python.TestPythonController")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in" "${CMAKE_CURRENT_BINARY_DIR}/Test${PYTHON}/mc_rtc-Test${PYTHON}.conf")
    add_test(NAME "Test${PYTHON}" COMMAND test_controller_ticker --run_test=RUN "${CMAKE_CURRENT_BINARY_DIR}/Test${PYTHON}/mc_rtc-Test${PYTHON}.conf" 400 "" "${CMAKE_CURRENT_SOURCE_DIR}/test_python")
    add_test(Test${PYTHON}Log "${CMAKE_CURRENT_SOURCE_DIR}/check_log.py" "${TEST_CONTROLLER_NAME}" 400 "${UTILS_BIN_DIR}")
  endforeach()
endif()
