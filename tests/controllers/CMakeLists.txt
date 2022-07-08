#
# Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(LOG_ENABLED "false")
set(LOG_POLICY "non-threaded")
set(ENABLED_OBSERVERS "")
set(RUN_OBSERVERS "")
set(UPDATE_OBSERVERS "")
if(WIN32)
  set(LOG_DIRECTORY "$ENV{TEMP}")
  string(REPLACE "\\" "/" LOG_DIRECTORY "${LOG_DIRECTORY}")
else()
  set(LOG_DIRECTORY "/tmp")
endif()

macro(mc_rtc_add_test)
cmake_parse_arguments(MC_RTC_TEST "" "NAME;COMMAND" "" ${ARGN} )
string(REPLACE ";" " " CMD_ARGS "${MC_RTC_TEST_UNPARSED_ARGUMENTS}") #Not robust to having ';' inside an arg. Could use list(JOIN ...) in CMake>=3.12
add_test(${ARGN})

# Adding a project configuration file (for MSVC only)
string(REPLACE "<CONFIG>" "(Configuration)" CMD_ARGS ${CMD_ARGS})
string(REPLACE "<CONFIGURATION>" "(Configuration)" CMD_ARGS ${CMD_ARGS})
generate_msvc_dot_user_file(NAME ${MC_RTC_TEST_NAME}
                            COMMAND "$(SolutionDir)tests/$(Configuration)/${MC_RTC_TEST_COMMAND}.exe"
                            COMMAND_ARGS ${CMD_ARGS})
endmacro()

macro(controller_test_common NAME)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.in.cpp)
    set(${NAME}_SRC ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/${NAME}.cpp)
    configure_file(${NAME}.in.cpp ${${NAME}_SRC})
  else()
    set(${NAME}_SRC ${NAME}.cpp)
  endif()
  add_library(${NAME} SHARED ${${NAME}_SRC})
  target_link_libraries(${NAME} PUBLIC mc_control Boost::unit_test_framework)
  if(NOT Boost_USE_STATIC_LIBS)
    target_link_libraries(${NAME} PUBLIC Boost::dynamic_linking)
  endif()
  target_include_directories(${NAME} PRIVATE ${CMAKE_SOURCE_DIR}/tests)
  set(CONFIG_HEADER_INCLUDE_DIR "${CMAKE_BINARY_DIR}/tests/include/$<CONFIGURATION>")
  target_include_directories(${NAME} PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})
  set_target_properties(${NAME} PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${NAME}
    )
  if(ENABLE_FAST_TESTS)
    target_compile_definitions(${NAME} PRIVATE "-DMC_RTC_FAST_TESTS")
  endif()
  set(TEST_CONTROLLER_NAME ${NAME})
  set(OBSERVER_MODULE_PATH "${CMAKE_BINARY_DIR}/src/mc_observers")
  set(PYTHON_CONTROLLER_PATH "${CMAKE_BINARY_DIR}/src/mc_control/python")
  set(ROBOT_MODULE_PATH "${CMAKE_BINARY_DIR}/src/mc_robots")
  set(TEST_CONTROLLER_PATH "${CMAKE_CURRENT_BINARY_DIR}/${TEST_CONTROLLER_NAME}")
  if(CMAKE_CONFIGURATION_TYPES)
    set(OBSERVER_MODULE_PATH "${OBSERVER_MODULE_PATH}/$<CONFIGURATION>")
    set(PYTHON_CONTROLLER_PATH "${PYTHON_CONTROLLER_PATH}/$<CONFIGURATION>")
    set(ROBOT_MODULE_PATH "${ROBOT_MODULE_PATH}/$<CONFIGURATION>")
    set(TEST_CONTROLLER_PATH "${TEST_CONTROLLER_PATH}/$<CONFIGURATION>")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
endmacro()

macro(controller_test_construction_failure NAME)
  controller_test_common(${NAME})
  if(CMAKE_CONFIGURATION_TYPES)
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIGURATION>/mc_rtc-${NAME}.conf")
    file(GENERATE
       OUTPUT ${CONFIG_OUT}
       INPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  else()
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf")
  endif()
  set_target_properties(${NAME} PROPERTIES FOLDER tests/controllers/failure)
  mc_rtc_add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=CONSTRUCTION_FAILURE -- ${CONFIG_OUT})
endmacro()

macro(controller_test_run NAME NRITER)
  controller_test_common(${NAME})
  macro(setup_test_run VARIANT)
    if(CMAKE_CONFIGURATION_TYPES)
      set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${VARIANT}/$<CONFIG>/mc_rtc-${VARIANT}.conf")
      file(GENERATE
         OUTPUT ${CONFIG_OUT}
         INPUT ${CMAKE_CURRENT_BINARY_DIR}/${VARIANT}/mc_rtc-${VARIANT}.conf)
    else()
      set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${VARIANT}/mc_rtc-${VARIANT}.conf")
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.in.yaml")
      configure_file(${NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/etc/${VARIANT}.cmake.yaml")
      if(CMAKE_CONFIGURATION_TYPES)
        file(GENERATE
          OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIG>/etc/${VARIANT}.yaml"
          INPUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/etc/${VARIANT}.cmake.yaml")
      else()
        file(GENERATE
          OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/etc/${VARIANT}.yaml"
          INPUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/etc/${VARIANT}.cmake.yaml")
      endif()
    endif()
    if(TARGET ${VARIANT})
      set_target_properties(${VARIANT} PROPERTIES FOLDER tests/controllers/run/details)
    endif()
    if(ENABLE_FAST_TESTS)
      set(ACTUALITER 5)
    else()
      set(ACTUALITER ${NRITER})
    endif()
    mc_rtc_add_test(NAME ${VARIANT} COMMAND test_controller_ticker --run_test=RUN -- ${CONFIG_OUT} ${ACTUALITER})
    # Add a custom target to run the controller in real-time and to run the test in IDE
    add_custom_target(
      controller-test-run-${VARIANT}
      COMMAND test_controller_ticker --run_test=RUN -- ${CONFIG_OUT} ${NRITER} "" 1
      DEPENDS ${NAME}
      VERBATIM
      USES_TERMINAL
    )
    set_target_properties(controller-test-run-${VARIANT} PROPERTIES FOLDER tests/controllers/run)
  endmacro()
  setup_test_run(${NAME})
endmacro()

function(setup_test_log_fixture CTL_TEST LOG_TEST)
  set_tests_properties(${CTL_TEST} PROPERTIES FIXTURES_SETUP ${CTL_TEST} FIXTURES_REQUIRED "${LOG_TEST}")
  foreach(L_TEST ${LOG_TEST})
    set_tests_properties(${L_TEST} PROPERTIES FIXTURES_CLEANUP ${L_TEST} FIXTURES_REQUIRED ${CTL_TEST})
  endforeach()
endfunction()

controller_test_construction_failure(NotAController)
controller_test_construction_failure(NoDestroyController)
controller_test_construction_failure(NoCreateController)

set(LOG_ENABLED "true")
controller_test_run(TestPostureController 1000)

# These tests do not work when building in a chroot such as during Debian packaging
if(NOT DEFINED PYTHON_DEB_ROOT)
  set(UTILS_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/../../utils)
  if(CMAKE_CONFIGURATION_TYPES)
    set(UTILS_BIN_DIR "${UTILS_BIN_DIR}/$<CONFIGURATION>")
  endif()
  if(ENABLE_FAST_TESTS)
    set(NRITER 5)
  else()
    set(NRITER 1000)
  endif()
  add_test(NAME TestPostureControllerLog COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/check_log.py" "TestPostureController" ${NRITER} ${UTILS_BIN_DIR})
  setup_test_log_fixture(TestPostureController TestPostureControllerLog)

  # Test threaded log policy
  set(LOG_POLICY "threaded")
  set(CONFIG_OUT ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/mc_rtc-TestPostureController-threaded-log.conf)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CONFIG_OUT})
  if(CMAKE_CONFIGURATION_TYPES)
    set(CONFIG_IN "${CONFIG_OUT}")
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/$<CONFIGURATION>/mc_rtc-TestPostureController-threaded-log.conf")
    file(GENERATE OUTPUT "${CONFIG_OUT}" INPUT "${CONFIG_IN}")
  endif()
  if(ENABLE_FAST_TESTS)
    set(NRITER 5)
  else()
    set(NRITER 1500)
  endif()
  add_test(NAME TestPostureControllerThreaded COMMAND test_controller_ticker --run_test=RUN -- "${CONFIG_OUT}" ${NRITER})
  add_test(NAME TestPostureControllerThreadedLog COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/check_log.py" "TestPostureController" ${NRITER} "${UTILS_BIN_DIR}")
  setup_test_log_fixture(TestPostureControllerThreaded TestPostureControllerThreadedLog)

  # Test controller switch and threaded log policy
  add_library(TestPostureController2 SHARED TestPostureController2.cpp)
  set_target_properties(TestPostureController2 PROPERTIES FOLDER tests/controllers/run)
  target_link_libraries(TestPostureController2 PUBLIC ${Boost_LIBRARIES} mc_control)
  set_target_properties(TestPostureController2 PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
    PREFIX ""
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController
    )
  set(TEST_CONTROLLER_NAME "TestPostureController\", \"TestPostureController2")
  set(CONFIG_OUT ${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/mc_rtc-TestPostureController2.conf)
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CONFIG_OUT})
  if(CMAKE_CONFIGURATION_TYPES)
    set(CONFIG_IN "${CONFIG_OUT}")
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/TestPostureController/$<CONFIGURATION>/mc_rtc-TestPostureController2.conf")
    file(GENERATE OUTPUT "${CONFIG_OUT}" INPUT "${CONFIG_IN}")
  endif()
  if(ENABLE_FAST_TESTS)
    set(NRITER 5)
    target_compile_definitions(TestPostureController2 PRIVATE "-DMC_RTC_FAST_TESTS")
  else()
    set(NRITER 1200)
  endif()
  mc_rtc_add_test(NAME TestControllerSwitch COMMAND test_controller_ticker --run_test=RUN -- "${CONFIG_OUT}" ${NRITER} TestPostureController2)
  add_test(NAME TestControllerSwitchLog1 COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/check_log.py" "TestPostureController" ${NRITER} "${UTILS_BIN_DIR}")
  add_test(NAME TestControllerSwitchLog2 COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/check_log.py" "TestPostureController2" ${NRITER} "${UTILS_BIN_DIR}")
  setup_test_log_fixture(TestControllerSwitch "TestControllerSwitchLog1;TestControllerSwitchLog2")
endif()

set(LOG_ENABLED "false")
set(LOG_POLICY "non-threaded")
# mc_task test controllers
controller_test_run(TestCoMTaskController 4001)
controller_test_run(TestMomentumTaskController 2001)
controller_test_run(TestGazeTaskController 501)
controller_test_run(TestPBVSTaskController 1001)
controller_test_run(TestPositionTaskController 5000)
controller_test_run(TestOrientationTaskController 6001)
controller_test_run(TestEndEffectorTaskController 4001)
controller_test_run(TestVectorOrientationTaskController 2001)
# mc_solver test controllers
controller_test_run(TestCoMInBoxController 4001)
controller_test_run(TestCollisionController 2001)

# Test FSM generic state configuration option
set(FSM_STATES_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/src/mc_control/fsm/states/")
if(CMAKE_CONFIGURATION_TYPES)
  set(FSM_STATES_INSTALL_PREFIX "${FSM_STATES_INSTALL_PREFIX}/$<CONFIGURATION>")
endif()
controller_test_run(TestFSMStateOptions 100)
target_link_libraries(TestFSMStateOptions PUBLIC mc_control_fsm)

set(FSM_TEST_STATES_DIR "${CMAKE_BINARY_DIR}/tests/fsm_states/TestMetaContinuity")
if(CMAKE_CONFIGURATION_TYPES)
  set(FSM_TEST_STATES_DIR "${FSM_TEST_STATES_DIR}/$<CONFIGURATION>")
endif()
controller_test_run(TestFSMMetaContinuity 20)
target_link_libraries(TestFSMMetaContinuity PUBLIC mc_control_fsm)

set(ENABLED_OBSERVERS "\"Encoder\", \"BodySensor\", \"KinematicInertial\"")
set(RUN_OBSERVERS "\"Encoder\", \"BodySensor\"")
set(UPDATE_OBSERVERS "\"Encoder\"")

# mc_observer test controllers
file(READ "ObserverPipelines.json" OBSERVER_PIPELINES)
controller_test_run(TestObserverController 2001)
target_link_libraries(TestObserverController PUBLIC BodySensorObserver)
unset(OBSERVER_PIPELINES)

# Test run of sample controllers
macro(controller_sample_test_run NAME PATH_SUFFIX NRITER)
  set(TEST_CONTROLLER_NAME ${NAME})
  set(OBSERVER_MODULE_PATH "${PROJECT_BINARY_DIR}/src/mc_observers")
  set(PYTHON_CONTROLLER_PATH "${PROJECT_BINARY_DIR}/src/mc_control/python")
  set(ROBOT_MODULE_PATH "${PROJECT_BINARY_DIR}/src/mc_robots")
  set(TEST_CONTROLLER_PATH "${PROJECT_BINARY_DIR}/src/mc_control/samples/${NAME}${PATH_SUFFIX}")
  if(CMAKE_CONFIGURATION_TYPES)
    set(OBSERVER_MODULE_PATH "${OBSERVER_MODULE_PATH}/$<CONFIGURATION>")
    set(PYTHON_CONTROLLER_PATH "${PYTHON_CONTROLLER_PATH}/$<CONFIGURATION>")
    set(ROBOT_MODULE_PATH "${ROBOT_MODULE_PATH}/$<CONFIGURATION>")
    set(TEST_CONTROLLER_PATH "${TEST_CONTROLLER_PATH}/$<CONFIGURATION>")
  endif()
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)

  if(CMAKE_CONFIGURATION_TYPES)
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIGURATION>/mc_rtc-${NAME}.conf")
    file(GENERATE
       OUTPUT ${CONFIG_OUT}
       INPUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf)
  else()
    set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${NAME}/mc_rtc-${NAME}.conf")
  endif()
  if(ENABLE_FAST_TESTS)
    set(ACTUALITER 5)
  else()
    set(ACTUALITER ${NRITER})
  endif()
  add_test(NAME "TestSampleController_${NAME}" COMMAND test_controller_ticker --run_test=RUN -- ${CONFIG_OUT} ${ACTUALITER})
endmacro()

controller_sample_test_run(CoM "" 1000)
controller_sample_test_run(EndEffector "" 1000)
controller_sample_test_run(Posture "" 1000)

if(ENABLE_FAST_TESTS)
  set(ACTUALITER 5)
else()
  set(ACTUALITER 20)
endif()
add_test(NAME "test_controller_restart" COMMAND test_controller_restart ${CONFIG_OUT} ${ACTUALITER})

controller_sample_test_run(Text "" 1000)
file(GENERATE OUTPUT "${TEST_CONTROLLER_PATH}/etc/Text.conf" INPUT "${PROJECT_SOURCE_DIR}/src/mc_control/samples/Text/etc/Text.conf")

# Test sample FSM controller
controller_sample_test_run(FSM "/src" 8000)
set(AROBASE "@")
set(MC_FSM_STATES_RUNTIME_DESTINATION_PREFIX "${PROJECT_BINARY_DIR}/src/mc_control/fsm/states/")
if(CMAKE_CONFIGURATION_TYPES)
  set(MC_FSM_STATES_RUNTIME_DESTINATION_PREFIX "${MC_FSM_STATES_RUNTIME_DESTINATION_PREFIX}/$<CONFIGURATION>")
endif()
set(MC_FSM_STATES_DATA_DESTINATION_PREFIX "${PROJECT_SOURCE_DIR}/src/mc_control/fsm/states/data/")
configure_file("${PROJECT_SOURCE_DIR}/src/mc_control/samples/FSM/etc/FSM.in.conf" "${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf")
file(GENERATE OUTPUT "${TEST_CONTROLLER_PATH}/etc/FSM.conf" INPUT "${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf")
unset(AROBASE)

# Test LIPMStabilizer controller execution
# Runs open-loop, does not check stability, only that it is runnable
controller_sample_test_run(LIPMStabilizer "/src" 1000)
set(AROBASE "@")
set(LIPMStabilizer_STATES_DESTINATION_PREFIX "${PROJECT_BINARY_DIR}/src/mc_control/fsm/states/")
if(CMAKE_CONFIGURATION_TYPES)
  set(LIPMStabilizer_STATES_DESTINATION_PREFIX "${LIPMStabilizer_STATES_DESTINATION_PREFIX}/$<CONFIGURATION>")
endif()
set(LIPMStabilizer_STATES_DATA_DESTINATION_PREFIX "${PROJECT_SOURCE_DIR}/src/mc_control/fsm/states/data/")
set(LIPMStabilizer_INIT_STATE "LIPMStabilizer::Standing")
configure_file("${PROJECT_SOURCE_DIR}/src/mc_control/samples/LIPMStabilizer/etc/LIPMStabilizer.in.yaml" "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml")
file(GENERATE OUTPUT "${TEST_CONTROLLER_PATH}/etc/LIPMStabilizer.yaml" INPUT "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml")
unset(AROBASE)

set(ENABLED_OBSERVERS "")
set(RUN_OBSERVERS "")
set(UPDATE_OBSERVERS "")

# Create a test plugin
add_library(TestPlugin SHARED plugins/TestPlugin.cpp)
set_target_properties(TestPlugin PROPERTIES FOLDER tests/controllers/plugins)
target_link_libraries(TestPlugin PUBLIC mc_control)
set_target_properties(TestPlugin PROPERTIES
  COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
  PREFIX ""
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestPlugin"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestPlugin"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestPlugin"
)

# Test controller plugin, checks that
# - a provided global plugin (Plugin0) is loaded for both controllers
# - Plugin1 and Plugin2 are used in the first controller
# - Plugin2 and Plugin3 are used in the second controller
# - Plugin0 and Plugin2 are the same instance of the plugin
add_library(TestPluginController SHARED TestPluginController.cpp)
set_target_properties(TestPluginController PROPERTIES FOLDER tests/controllers/plugins)
target_link_libraries(TestPluginController PUBLIC Boost::unit_test_framework mc_control)
set_target_properties(TestPluginController PROPERTIES
  COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
  PREFIX ""
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestPluginController"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestPluginController"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/TestPluginController"
)
set(TEST_CONTROLLER_NAME "TestPluginController_1_2\", \"TestPluginController_2_3")
set(TEST_PLUGINS "\"Plugin0\"")
set(PLUGINS_MODULE_PATH "${CMAKE_CURRENT_BINARY_DIR}/TestPlugin")
foreach(I RANGE 0 3)
  file(WRITE "${PLUGINS_MODULE_PATH}/etc/Plugin${I}.yaml" "common: Plugin${I}")
  if(CMAKE_CONFIGURATION_TYPES)
    file(GENERATE OUTPUT "${PLUGINS_MODULE_PATH}/$<CONFIG>/etc/Plugin${I}.yaml"
                  INPUT  "${PLUGINS_MODULE_PATH}/etc/Plugin${I}.yaml")
  endif()
endforeach()
set(TEST_CONTROLLER_PATH "${CMAKE_CURRENT_BINARY_DIR}/TestPluginController")
set(TEST_CONTROLLER_PATH_BASE "${TEST_CONTROLLER_PATH}")
if(CMAKE_CONFIGURATION_TYPES)
  set(PLUGINS_MODULE_PATH "${PLUGINS_MODULE_PATH}/$<CONFIG>")
  set(TEST_CONTROLLER_PATH "${TEST_CONTROLLER_PATH}/$<CONFIG>")
endif()
set(CONFIG_OUT "${CMAKE_CURRENT_BINARY_DIR}/TestPluginController/mc_rtc-TestPluginController.conf")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in" ${CONFIG_OUT})
file(WRITE "${TEST_CONTROLLER_PATH_BASE}/etc/TestPluginController_1_2.yaml" "Plugins: [Plugin1, Plugin2]")
# We generate controller-level configuration for every plugins, this checks
# that this configuration is only loaded if the plugin is enabled at the global
# or controller level
foreach(I RANGE 0 3)
  file(WRITE "${TEST_CONTROLLER_PATH_BASE}/etc/TestPluginController_1_2/plugins/Plugin${I}.yaml" "TestPluginController_1_2: true\nexclusive_to_TestPluginController_1_2: true")
endforeach()
file(WRITE "${TEST_CONTROLLER_PATH_BASE}/etc/TestPluginController_2_3.yaml" "Plugins: [Plugin2, Plugin3]")
foreach(I RANGE 0 3)
  file(WRITE "${TEST_CONTROLLER_PATH_BASE}/etc/TestPluginController_2_3/plugins/Plugin${I}.yaml" "TestPluginController_2_3: true\nexclusive_to_TestPluginController_1_2: false")
endforeach()
if(CMAKE_CONFIGURATION_TYPES)
  set(CONFIG_IN "${CONFIG_OUT}")
  set(CONFIG_FOLDER_OUT "${CMAKE_CURRENT_BINARY_DIR}/TestPluginController/$<CONFIG>")
  set(CONFIG_OUT "${CONFIG_FOLDER_OUT}/mc_rtc-TestPluginController.conf")
  file(GENERATE OUTPUT "${CONFIG_OUT}" INPUT "${CONFIG_IN}")
  file(GENERATE
    OUTPUT "${CONFIG_FOLDER_OUT}/etc/TestPluginController_1_2.yaml"
    INPUT "${TEST_CONTROLLER_PATH_BASE}/etc/TestPluginController_1_2.yaml"
  )
  file(GENERATE
    OUTPUT "${CONFIG_FOLDER_OUT}/etc/TestPluginController_2_3.yaml"
    INPUT "${TEST_CONTROLLER_PATH_BASE}/etc/TestPluginController_2_3.yaml"
  )
  foreach(I RANGE 0 3)
    file(GENERATE
      OUTPUT "${CONFIG_FOLDER_OUT}/etc/TestPluginController_1_2/plugins/Plugin${I}.yaml"
      INPUT "${TEST_CONTROLLER_PATH_BASE}/etc/TestPluginController_1_2/plugins/Plugin${I}.yaml")
    file(GENERATE
      OUTPUT "${CONFIG_FOLDER_OUT}/etc/TestPluginController_2_3/plugins/Plugin${I}.yaml"
      INPUT "${TEST_CONTROLLER_PATH_BASE}/etc/TestPluginController_2_3/plugins/Plugin${I}.yaml")
  endforeach()
endif()
mc_rtc_add_test(NAME TestPluginController COMMAND test_controller_ticker --run_test=RUN -- "${CONFIG_OUT}" 5 TestPluginController_2_3)

# Test Python bindings
if(PYTHON_BINDING)
  file(READ "ObserverPipelinesPython.json" OBSERVER_PIPELINES)
  set(LOG_ENABLED "true")
  get_python_names(python_names)
  foreach(PYTHON ${python_names})
    set(TEST_CONTROLLER_NAME "${PYTHON}#test_python.TestPythonController")
    set(PYTHON_CONTROLLER_PATH "${CMAKE_BINARY_DIR}/src/mc_control/python")
    set(ROBOT_MODULE_PATH "${CMAKE_BINARY_DIR}/src/mc_robots")
    set(TEST_CONTROLLER_PATH "${CMAKE_CURRENT_BINARY_DIR}/${TEST_CONTROLLER_NAME}")
    if(CMAKE_CONFIGURATION_TYPES)
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
    if(CMAKE_CONFIGURATION_TYPES)
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
    set(LDPATH_CONTROL "$<TARGET_FILE_DIR:mc_control>")
    if(ENABLE_FAST_TESTS)
      set(NRITER 5)
    else()
      set(NRITER 400)
    endif()
    add_test(NAME "Test${PYTHON}Controller" COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${PYTHONPATH}${PATH_SEP}$ENV{PYTHONPATH}" ${CMAKE_COMMAND} -E env "${LDPATH_VAR}=${LDPATH}${PATH_SEP}${LDPATH_CONTROL}${PATH_SEP}$ENV{${LDPATH_VAR}}" $<TARGET_FILE:test_controller_ticker> --run_test=RUN -- "${CONFIG_OUT}" ${NRITER} "")
    if(NOT DEFINED PYTHON_DEB_ROOT)
      add_test(NAME Test${PYTHON}Log COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/check_log.py" "${TEST_CONTROLLER_NAME}" ${NRITER} "${UTILS_BIN_DIR}")
      setup_test_log_fixture(Test${PYTHON}Controller Test${PYTHON}Log)
    endif()
  endforeach()
  unset(OBSERVER_PIPELINES)
endif()
