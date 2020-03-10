#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

find_package(Boost REQUIRED COMPONENTS unit_test_framework timer)

############################
# -- Compile-time tests -- #
############################

add_executable(LoggerTraits LoggerTraits.cpp)
set_target_properties(LoggerTraits PROPERTIES FOLDER tests/compilation)
target_link_libraries(LoggerTraits PUBLIC mc_control)

add_executable(testConstraintImplementation testConstraintImplementation.cpp)
set_target_properties(testConstraintImplementation PROPERTIES FOLDER tests/compilation)
target_link_libraries(testConstraintImplementation PUBLIC mc_solver)

#################################
#      -- Utility tests --      #
# -- FIXME not actuals tests -- #
#################################

add_executable(testBSplineTrajectory testBSplineTrajectory.cpp)
set_target_properties(testBSplineTrajectory PROPERTIES FOLDER tests/utility)
target_link_libraries(testBSplineTrajectory PUBLIC mc_trajectory mc_rtc_gui)

add_executable(testQuadraticGenerator testQuadraticGenerator.cpp)
set_target_properties(testQuadraticGenerator PROPERTIES FOLDER tests/utility)
target_link_libraries(testQuadraticGenerator PUBLIC mc_rbdyn)

#################################
# -- Framework utility tests -- #
#################################

set(ROBOTS_BUILD_DIR src/mc_robots)
if(CMAKE_CONFIGURATION_TYPES)
  set(ROBOTS_BUILD_DIR "${ROBOTS_BUILD_DIR}/$<CONFIGURATION>")
endif()
set(CONFIG_HEADER_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/$<CONFIGURATION>")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests_config.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/tests_config.cmake.h)
file(GENERATE
     OUTPUT "${CONFIG_HEADER_INCLUDE_DIR}/tests_config.h"
     INPUT  "${CMAKE_CURRENT_BINARY_DIR}/tests_config.cmake.h")

MACRO(mc_rtc_test NAME)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cpp)
    add_executable(${NAME} ${NAME}.cpp)
  elseif(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.in.cpp)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.in.cpp
                   ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp)
    add_executable(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp)
  else()
    message(FATAL_ERROR "Cannot add mc_rtc_test: neither ${NAME}.cpp nor ${NAME}.in.cpp exists in ${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
  target_link_libraries(${NAME} PUBLIC Boost::unit_test_framework Boost::disable_autolinking ${ARGN})
  target_compile_definitions(${NAME} PRIVATE -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
  target_include_directories(${NAME} PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})
  set_target_properties(${NAME} PROPERTIES FOLDER tests/framework)
  add_test(${NAME} ${NAME})
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${NAME})
ENDMACRO()

mc_rtc_test(testConfiguration mc_rtc_utils mc_rbdyn)
mc_rtc_test(testGUIStateBuilder mc_rtc_gui)
mc_rtc_test(testJsonIO mc_rtc_utils mc_rbdyn)
mc_rtc_test(testConstraintSetLoader mc_solver)
mc_rtc_test(testMetaTaskLoader mc_tasks)
mc_rtc_test(testSolverTaskStorage mc_tasks)
mc_rtc_test(testCompletionCriteria mc_control)
mc_rtc_test(testSimulationContactPair mc_control)
mc_rtc_test(testDataStore mc_rtc_utils mc_rbdyn)

###########################
# -- FSM related tests -- #
###########################

add_subdirectory(fsm_states)

mc_rtc_test(testFSMStateFactory mc_control_fsm ConfigureState)
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/testFSMStateFactoryConfig.in.h"
               "${CMAKE_CURRENT_BINARY_DIR}/testFSMStateFactoryConfig.cmake.h")
file(GENERATE
     OUTPUT "${CONFIG_HEADER_INCLUDE_DIR}/testFSMStateFactoryConfig.h"
     INPUT  "${CMAKE_CURRENT_BINARY_DIR}/testFSMStateFactoryConfig.cmake.h")

######################################
# -- Controllers and robots tests -- #
######################################

add_executable(test_controller_ticker test_controller_ticker.cpp)
set_target_properties(test_controller_ticker PROPERTIES FOLDER tests/ticker)
target_link_libraries(test_controller_ticker PUBLIC mc_control Boost::unit_test_framework Boost::disable_autolinking)
target_compile_definitions(test_controller_ticker PRIVATE -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
# Adding a project configuration file (for MSVC only)
generate_msvc_dot_user_file(test_controller_ticker)

if(NOT DISABLE_CONTROLLER_TESTS)
  add_subdirectory(controllers)
endif()

if(NOT DISABLE_ROBOT_TESTS)
  add_subdirectory(robots)
endif()

#######################
# -- mc_rbdyn test -- #
#######################
mc_rtc_test(test_mc_rbdyn mc_rbdyn)

######################
# -- Network test -- #
######################

add_executable(dummyClient dummyClient.cpp)
set_target_properties(dummyClient PROPERTIES FOLDER tests/GUI)
target_link_libraries(dummyClient PUBLIC mc_control_client)

add_executable(dummyServer dummyServer.cpp)
set_target_properties(dummyServer PROPERTIES FOLDER tests/GUI)
target_link_libraries(dummyServer PUBLIC mc_control)

######################
# -- Filters test -- #
######################
add_executable(test_filters test_filters.cpp)
target_link_libraries(test_filters PUBLIC mc_filter SpaceVecAlg::SpaceVecAlg Boost::unit_test_framework Boost::disable_autolinking)
target_compile_definitions(test_filters PRIVATE -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
