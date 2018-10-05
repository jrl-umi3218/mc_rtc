find_package(Boost REQUIRED COMPONENTS unit_test_framework timer)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
if(WIN32)
  add_definitions( -DBOOST_ALL_DYN_LINK )
endif()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/tests_config.h.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/tests_config.h")
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

############################
# -- Compile-time tests -- #
############################

add_executable(LoggerTraits LoggerTraits.cpp)
target_link_libraries(LoggerTraits PUBLIC mc_control)

#################################
#      -- Utility tests --      #
# -- FIXME not actuals tests -- #
#################################

add_executable(testBSplineTrajectory testBSplineTrajectory.cpp)
target_link_libraries(testBSplineTrajectory PUBLIC mc_trajectory)

add_executable(testQuadraticGenerator testQuadraticGenerator.cpp)
pkg_config_use_dependency(testQuadraticGenerator Tasks)
pkg_config_use_dependency(testQuadraticGenerator mc_rbdyn_urdf)
target_link_libraries(testQuadraticGenerator PUBLIC mc_rbdyn)

#################################
# -- Framework utility tests -- #
#################################

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
  target_link_libraries(${NAME} PUBLIC ${Boost_LIBRARIES} ${ARGN})
  add_test(${NAME} ${NAME})
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${NAME})
ENDMACRO()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests_config.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/include/tests_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

mc_rtc_test(testConfiguration mc_rtc_utils mc_rbdyn)
mc_rtc_test(testGUIStateBuilder mc_rtc_gui)
mc_rtc_test(testJsonIO mc_rtc_utils mc_rbdyn)
mc_rtc_test(testConstraintSetLoader mc_solver)
mc_rtc_test(testMetaTaskLoader mc_tasks)
mc_rtc_test(testSolverTaskStorage mc_tasks)
mc_rtc_test(testCompletionCriteria mc_control)
mc_rtc_test(testSimulationContactPair mc_control)

###########################
# -- FSM related tests -- #
###########################

add_subdirectory(fsm_states)

mc_rtc_test(testFSMStateFactory mc_control_fsm ConfigureState)

######################################
# -- Controllers and robots tests -- #
######################################

add_executable(test_controller_ticker test_controller_ticker.cpp)
pkg_config_use_dependency(test_controller_ticker mc_rbdyn_urdf)
target_link_libraries(test_controller_ticker PUBLIC ${Boost_LIBRARIES} mc_control)
pkg_config_use_dependency(test_controller_ticker Tasks)
# Adding a project configuration file (for MSVC only)
generate_msvc_dot_user_file(test_controller_ticker)

if(NOT DISABLE_CONTROLLER_TESTS)
  add_subdirectory(controllers)
endif()

if(NOT DISABLE_ROBOT_TESTS)
  add_subdirectory(robots)
endif()

######################
# -- Network test -- #
######################

add_executable(dummyClient dummyClient.cpp)
target_link_libraries(dummyClient PUBLIC mc_control_client)

add_executable(dummyServer dummyServer.cpp)
target_link_libraries(dummyServer PUBLIC mc_control)
