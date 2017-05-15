add_executable(testBSplineTrajectory testBSplineTrajectory.cpp)
target_link_libraries(testBSplineTrajectory mc_trajectory)

add_executable(testQuadraticGenerator testQuadraticGenerator.cpp)
pkg_config_use_dependency(testQuadraticGenerator Tasks)
target_link_libraries(testQuadraticGenerator mc_rbdyn)

find_package(Boost REQUIRED COMPONENTS unit_test_framework timer)
include_directories(${Boost_INCLUDE_DIRS})
add_definitions(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
if(WIN32)
  add_definitions( -DBOOST_ALL_DYN_LINK )
endif()

MACRO(mc_rtc_test NAME)
  add_executable(${NAME} ${NAME}.cpp)
  target_link_libraries(${NAME} ${Boost_LIBRARIES} ${ARGN})
  add_test(${NAME} ${NAME})
  # Adding a project configuration file (for MSVC only)
  generate_msvc_dot_user_file(${NAME})
ENDMACRO()

mc_rtc_test(testConfiguration mc_rtc_utils)

add_executable(test_controller_ticker test_controller_ticker.cpp)
pkg_config_use_dependency(test_controller_ticker mc_rbdyn_urdf_cpp)
target_link_libraries(test_controller_ticker ${Boost_LIBRARIES} mc_control)
pkg_config_use_dependency(test_controller_ticker Tasks)
# Adding a project configuration file (for MSVC only)
generate_msvc_dot_user_file(test_controller_ticker)

add_subdirectory(controllers)
add_subdirectory(robots)
