#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

find_package(Boost REQUIRED COMPONENTS unit_test_framework timer)

# ######################################################################################
# -- Compile-time tests -- #
# ######################################################################################

add_executable(LoggerTraits LoggerTraits.cpp)
set_target_properties(LoggerTraits PROPERTIES FOLDER tests/compilation)
target_link_libraries(LoggerTraits PUBLIC mc_control)

add_executable(testConstraintImplementation testConstraintImplementation.cpp)
set_target_properties(testConstraintImplementation PROPERTIES FOLDER tests/compilation)
target_link_libraries(testConstraintImplementation PUBLIC mc_solver)

# ######################################################################################
# -- Utility tests --      # -- FIXME not actuals tests -- #
# ######################################################################################

add_executable(testBSplineTrajectory testBSplineTrajectory.cpp)
set_target_properties(testBSplineTrajectory PROPERTIES FOLDER tests/utility)
target_link_libraries(testBSplineTrajectory PUBLIC mc_trajectory mc_rtc_gui)

add_executable(testQuadraticGenerator testQuadraticGenerator.cpp)
set_target_properties(testQuadraticGenerator PROPERTIES FOLDER tests/utility)
target_link_libraries(testQuadraticGenerator PUBLIC mc_rbdyn)

add_executable(testNotify testNotify.cpp)
set_target_properties(testNotify PROPERTIES FOLDER tests/utility)
target_link_libraries(testNotify PUBLIC mc_rtc_utils)

# ######################################################################################
# -- Framework utility tests -- #
# ######################################################################################

set(ROBOTS_BUILD_DIR src/mc_robots)
if(CMAKE_CONFIGURATION_TYPES)
  set(ROBOTS_BUILD_DIR "${ROBOTS_BUILD_DIR}/$<CONFIGURATION>")
endif()
set(CONFIG_HEADER_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/$<CONFIGURATION>")
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/tests_config.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/tests_config.cmake.h
)
file(
  GENERATE
  OUTPUT "${CONFIG_HEADER_INCLUDE_DIR}/tests_config.h"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/tests_config.cmake.h"
)

macro(mc_rtc_test NAME)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cpp)
    add_executable(${NAME} ${NAME}.cpp)
  elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.in.cpp)
    configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.in.cpp
      ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp
    )
    add_executable(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp)
  else()
    message(
      FATAL_ERROR
        "Cannot add mc_rtc_test: neither ${NAME}.cpp nor ${NAME}.in.cpp exists in ${CMAKE_CURRENT_SOURCE_DIR}"
    )
  endif()
  target_link_libraries(
    ${NAME} PUBLIC Boost::unit_test_framework Boost::disable_autolinking ${ARGN}
  )
  if(NOT Boost_USE_STATIC_LIBS)
    target_link_libraries(${NAME} PUBLIC Boost::dynamic_linking)
  endif()
  target_compile_definitions(${NAME} PRIVATE -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
  target_include_directories(${NAME} PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})
  target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  set_target_properties(${NAME} PROPERTIES FOLDER tests/framework)
  add_test(NAME ${NAME} COMMAND ${NAME})
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${NAME})
endmacro()

mc_rtc_test(testConfiguration mc_rtc_utils mc_rbdyn)
mc_rtc_test(testSchema mc_rtc_utils mc_rbdyn)
mc_rtc_test(testGUIStateBuilder mc_rtc_gui)
mc_rtc_test(testJsonIO mc_rtc_utils mc_rbdyn)
mc_rtc_test(testConstraintSetLoader mc_solver)
mc_rtc_test(testMetaTaskLoader mc_tasks)
mc_rtc_test(testSolverTaskStorage mc_tasks)
mc_rtc_test(testCompletionCriteria mc_control)
mc_rtc_test(testSimulationContactPair mc_control)
mc_rtc_test(testDataStore mc_rtc_utils mc_rbdyn)
mc_rtc_test(test_mc_rtc_utils mc_rtc_utils)
mc_rtc_test(testConfigurationHelpers mc_rtc_utils)
mc_rtc_test(test_io_utils mc_rtc_utils)
get_filename_component(
  EXAMPLE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../doc/_examples" ABSOLUTE
)
mc_rtc_test(testSchemaExamples mc_tasks)
mc_rtc_test(testLogger mc_rbdyn)
mc_rtc_test(testLogUtils mc_rbdyn)
mc_rtc_test(testRobotModule mc_rbdyn)
find_description_package(jvrc_description)
set_target_properties(
  testRobotModule PROPERTIES COMPILE_FLAGS
                             "-DJVRC_DESCRIPTION_PATH=\"${JVRC_DESCRIPTION_PATH}\""
)
mc_rtc_test(testCanonicalRobot mc_rbdyn)
mc_rtc_test(testSolverBackend mc_tasks)

# ######################################################################################
# -- FSM related tests -- #
# ######################################################################################

add_subdirectory(fsm_states)

mc_rtc_test(testFSMStateFactory mc_control_fsm ConfigureState)
# Run in ConfigureState dir to find the DLL on Windows
set_tests_properties(
  testFSMStateFactory PROPERTIES WORKING_DIRECTORY "$<TARGET_FILE_DIR:ConfigureState>"
)
# Generate an MSVC .user file with the right path
generate_msvc_dot_user_file(
  testFSMStateFactory "$(SolutionDir)/tests/fsm_states/ConfigureState/$(Configuration)"
)
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/testFSMStateFactoryConfig.in.h"
  "${CMAKE_CURRENT_BINARY_DIR}/testFSMStateFactoryConfig.cmake.h"
)
file(
  GENERATE
  OUTPUT "${CONFIG_HEADER_INCLUDE_DIR}/testFSMStateFactoryConfig.h"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/testFSMStateFactoryConfig.cmake.h"
)

# ######################################################################################
# -- Controllers and robots tests -- #
# ######################################################################################

function(add_global_controller_tester NAME)
  add_library(${NAME} OBJECT ${NAME}.cpp)
  set_target_properties(${NAME} PROPERTIES FOLDER tests/ticker)
  target_link_libraries(
    ${NAME} PUBLIC mc_control Boost::unit_test_framework Boost::disable_autolinking
  )
  if(NOT Boost_USE_STATIC_LIBS)
    target_link_libraries(${NAME} PUBLIC Boost::dynamic_linking)
  endif()
  target_compile_definitions(${NAME} PRIVATE -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
  target_include_directories(${NAME} PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})
endfunction()

add_global_controller_tester(test_global_controller_construction_failure)
add_global_controller_tester(test_global_controller_run)

function(
  add_global_controller_test
  NAME
  TEST_MAIN
  TEST_NAME
  CONFIGURATION_FILE
  RUN_ITER
  NEXT_CONTROLLER
)
  set(CONFIG_CPP_INT_OUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/config.in.cpp)
  set(CONFIG_CPP_OUT ${CMAKE_CURRENT_BINARY_DIR}/${NAME}/$<CONFIG>/config.cpp)
  configure_file(
    ${PROJECT_SOURCE_DIR}/tests/test_global_controller_config.in.cpp
    ${CONFIG_CPP_INT_OUT}
  )
  file(
    GENERATE
    OUTPUT ${CONFIG_CPP_OUT}
    INPUT ${CONFIG_CPP_INT_OUT}
  )
  add_executable(${TEST_NAME} ${CONFIG_CPP_OUT})
  target_link_libraries(${TEST_NAME} PUBLIC ${TEST_MAIN})
  add_test(NAME ${NAME} COMMAND ${ARGN} $<TARGET_FILE:${TEST_NAME}>)
  generate_msvc_dot_user_file(NAME ${TEST_NAME})
endfunction()

function(add_global_controller_test_construction_failure NAME CONFIGURATION_FILE)
  add_global_controller_test(
    ${NAME}
    test_global_controller_construction_failure
    ConstructionFailure_${NAME}
    ${CONFIGURATION_FILE}
    0
    ""
    ${ARGN}
  )
endfunction()

function(add_global_controller_test_run NAME CONFIGURATION_FILE NRITER NEXTCTL)
  add_global_controller_test(
    ${NAME}
    test_global_controller_run
    ControllerRun_${NAME}
    ${CONFIGURATION_FILE}
    ${NRITER}
    "${NEXTCTL}"
    ${ARGN}
  )
endfunction()

add_executable(test_controller_restart test_controller_restart.cpp)
set_target_properties(test_controller_restart PROPERTIES FOLDER tests/ticker)
target_link_libraries(test_controller_restart PUBLIC mc_control)
target_include_directories(test_controller_restart PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})
generate_msvc_dot_user_file(test_controller_restart)

if(NOT DISABLE_CONTROLLER_TESTS)
  add_subdirectory(controllers)
endif()

if(NOT DISABLE_ROBOT_TESTS)
  add_subdirectory(robots)
endif()

# ######################################################################################
# -- mc_rbdyn test -- #
# ######################################################################################
mc_rtc_test(test_mc_rbdyn mc_rbdyn)

# ######################################################################################
# -- Network test -- #
# ######################################################################################

add_executable(gui_SampleClient gui_SampleClient.cpp)
set_target_properties(gui_SampleClient PROPERTIES FOLDER tests/GUI)
target_link_libraries(gui_SampleClient PUBLIC mc_control_client)

add_executable(gui_SampleServer gui_SampleServer.cpp)
target_include_directories(gui_SampleServer PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})
set_target_properties(gui_SampleServer PROPERTIES FOLDER tests/GUI)
target_link_libraries(gui_SampleServer PUBLIC mc_control)

add_executable(gui_AdvancedForm gui_AdvancedForm.cpp)
target_include_directories(gui_AdvancedForm PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})
set_target_properties(gui_AdvancedForm PROPERTIES FOLDER tests/GUI)
target_link_libraries(gui_AdvancedForm PUBLIC mc_control)

add_executable(gui_Schema gui_Schema.cpp)
target_include_directories(gui_Schema PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})
set_target_properties(gui_Schema PROPERTIES FOLDER tests/GUI)
target_link_libraries(gui_Schema PUBLIC mc_control)

# ######################################################################################
# -- Filters test -- #
# ######################################################################################
mc_rtc_test(test_filters mc_filter SpaceVecAlg::SpaceVecAlg)

mc_rtc_test(test_interpolation mc_control)

add_subdirectory(global_controller_configuration)
