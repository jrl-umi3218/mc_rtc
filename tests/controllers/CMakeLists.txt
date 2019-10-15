#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(LOG_ENABLED "false")
set(LOG_POLICY "non-threaded")
set(USE_SANDBOX "true")
set(ENABLED_OBSERVERS "")
set(RUN_OBSERVERS "")
set(UPDATE_OBSERVERS "")

macro(controller_test_common NAME)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.in.cpp)
    set(${NAME}_SRC ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/${NAME}.cpp)
    configure_file(${NAME}.in.cpp ${${NAME}_SRC})
  else()
    set(${NAME}_SRC ${NAME}.cpp)
  endif()
  add_library(${NAME} SHARED ${${NAME}_SRC})
  target_link_libraries(${NAME} PUBLIC mc_control Boost::unit_test_framework)
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    )
  set(TEST_CONTROLLER_NAME ${NAME})
  set(OBSERVER_MODULE_PATH "${CMAKE_BINARY_DIR}/src/mc_observers")
  set(PYTHON_CONTROLLER_PATH "${CMAKE_BINARY_DIR}/src/mc_control/python")
  set(ROBOT_MODULE_PATH "${CMAKE_BINARY_DIR}/src/mc_robots")
  set(TEST_CONTROLLER_PATH "${CMAKE_CURRENT_BINARY_DIR}/${TEST_CONTROLLER_NAME}")
  if(NOT DEFINED CMAKE_BUILD_TYPE)
    set(OBSERVER_MODULE_PATH "${OBSERVER_MODULE_PATH}/$<CONFIGURATION>")
    set(PYTHON_CONTROLLER_PATH "${PYTHON_CONTROLLER_PATH}/$<CONFIGURATION>")
    set(ROBOT_MODULE_PATH "${ROBOT_MODULE_PATH}/$<CONFIGURATION>")
    set(TEST_CONTROLLER_PATH "${TEST_CONTROLLER_PATH}/$<CONFIGURATION>")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${NAME})
endmacro()

macro(controller_test_construction_failure NAME)
  controller_test_common(${NAME})
  if(DEFINED CMAKE_BUILD_TYPE)
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf")
  else()
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIGURATION>/mc_rtc-${NAME}.conf")
    file(GENERATE
       OUTPUT ${CONFIG_OUT}
       INPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  endif()
  set_target_properties(${NAME} PROPERTIES FOLDER tests/controllers/failure)
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=CONSTRUCTION_FAILURE -- ${CONFIG_OUT})
endmacro()

macro(controller_test_run NAME NRITER)
  controller_test_common(${NAME})
  if(DEFINED CMAKE_BUILD_TYPE)
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf")
  else()
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIGURATION>/mc_rtc-${NAME}.conf")
    file(GENERATE
       OUTPUT ${CONFIG_OUT}
       INPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  endif()
  set_target_properties(${NAME} PROPERTIES FOLDER tests/controllers/run)
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=RUN -- ${CONFIG_OUT} ${NRITER})
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
  add_test(NAME TestPostureControllerThreadedLogging COMMAND test_controller_ticker --run_test=RUN -- TestPostureController/mc_rtc-TestPostureController-threaded-log.conf 1500)
  add_test(TestPostureControllerThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController" 1500 "${UTILS_BIN_DIR}")

  # Test controller switch and threaded log policy
  add_library(TestPostureController2 SHARED TestPostureController2.cpp)
  set_target_properties(TestPostureController2 PROPERTIES FOLDER tests/controllers/run)
  target_link_libraries(TestPostureController2 PUBLIC ${Boost_LIBRARIES} mc_control)
  set_target_properties(TestPostureController2 PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController
    )
  set(TEST_CONTROLLER_NAME "TestPostureController\", \"TestPostureController2")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/mc_rtc-TestPostureController2.conf)
  add_test(NAME TestPostureController2 COMMAND test_controller_ticker --run_test=RUN -- TestPostureController/mc_rtc-TestPostureController2.conf 1200 TestPostureController2)
  add_test(TestPostureControllerControllerSwitchThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController" 1200 "${UTILS_BIN_DIR}")
  add_test(TestPostureController2ControllerSwitchThreadedLog ${CMAKE_CURRENT_SOURCE_DIR}/check_log.py "TestPostureController2" 1200 "${UTILS_BIN_DIR}")
endif()

set(LOG_POLICY "non-threaded")
# mc_task test controllers
controller_test_run(TestCoMTaskController 4001)
controller_test_run(TestPositionTaskController 5000)
controller_test_run(TestOrientationTaskController 6001)
controller_test_run(TestEndEffectorTaskController 4001)

set(ENABLED_OBSERVERS "\"Encoder\", \"BodySensor\", \"KinematicInertial\"")
set(RUN_OBSERVERS "\"Encoder\", \"BodySensor\"")
set(UPDATE_OBSERVERS "\"Encoder\"")

# mc_observer test controllers
controller_test_run(TestObserverController 2001)

set(ENABLED_OBSERVERS "")
set(RUN_OBSERVERS "")
set(UPDATE_OBSERVERS "")

# Test Python bindings
if(${PYTHON_BINDING})
  set(USE_SANDBOX "false")
  get_python_names(python_names)
  foreach(PYTHON ${python_names})
    set(TEST_CONTROLLER_NAME "${PYTHON}#test_python.TestPythonController")
    set(PYTHON_CONTROLLER_PATH "${CMAKE_BINARY_DIR}/src/mc_control/python")
    set(ROBOT_MODULE_PATH "${CMAKE_BINARY_DIR}/src/mc_robots")
    set(TEST_CONTROLLER_PATH "${CMAKE_CURRENT_BINARY_DIR}/${TEST_CONTROLLER_NAME}")
    if(NOT DEFINED CMAKE_BUILD_TYPE)
      set(PYTHON_CONTROLLER_PATH "${PYTHON_CONTROLLER_PATH}/$<CONFIGURATION>")
      set(ROBOT_MODULE_PATH "${ROBOT_MODULE_PATH}/$<CONFIGURATION>")
      set(TEST_CONTROLLER_PATH "${TEST_CONTROLLER_PATH}/$<CONFIGURATION>")
    endif()
    if(WIN32)
      set(PATH_SEP ";")
    else()
      set(PATH_SEP ":")
    endif()
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/Test${PYTHON}/mc_rtc-Test${PYTHON}.conf")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in" "${CONFIG_OUT}")
    if(NOT DEFINED CMAKE_BUILD_TYPE)
      set(CONFIG_IN "${CONFIG_OUT}")
      set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/Test${PYTHON}/$<CONFIGURATION>/mc_rtc-Test${PYTHON}.conf")
      file(GENERATE OUTPUT "${CONFIG_OUT}" INPUT "${CONFIG_IN}")
    endif()
    string(TOLOWER "${PYTHON}" PYTHON_LOWER)
    set(PYTHONPATH "${PROJECT_BINARY_DIR}/binding/python/mc_rtc/${PYTHON_LOWER}/$<CONFIGURATION>${PATH_SEP}${CMAKE_CURRENT_SOURCE_DIR}")
    if(WIN32)
      set(LDPATH_VAR "PATH")
    else()
      set(LDPATH_VAR "LD_LIBRARY_PATH")
    endif()
    set(LDPATH "$<TARGET_FILE_DIR:mc_control_fsm>")
    add_test(NAME "Test${PYTHON}Controller" COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${PYTHONPATH}${PATH_SEP}$ENV{PYTHONPATH}" ${CMAKE_COMMAND} -E env "${LDPATH_VAR}=${LDPATH}${PATH_SEP}$ENV{${LDPATH_VAR}}" $<TARGET_FILE:test_controller_ticker> --run_test=RUN -- "${CONFIG_OUT}" 400 "")
    if(UNIX)
      add_test(Test${PYTHON}Log "${CMAKE_CURRENT_SOURCE_DIR}/check_log.py" "${TEST_CONTROLLER_NAME}" 400 "${UTILS_BIN_DIR}")
    endif()
  endforeach()
endif()
