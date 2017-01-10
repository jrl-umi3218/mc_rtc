add_executable(testBSplineTrajectory testBSplineTrajectory.cpp)
target_link_libraries(testBSplineTrajectory mc_trajectory)

add_executable(testQuadraticGenerator testQuadraticGenerator.cpp)
pkg_config_use_dependency(testQuadraticGenerator Tasks)
target_link_libraries(testQuadraticGenerator mc_rbdyn)

FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework timer)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)
if(WIN32)
  ADD_DEFINITIONS( -DBOOST_ALL_DYN_LINK )
endif()

MACRO(MC_RTC_TEST NAME)
  add_executable(${NAME} ${NAME}.cpp)
  target_link_libraries(${NAME} ${Boost_LIBRARIES} ${ARGN})
  pkg_config_use_dependency(${NAME} jsoncpp)
  add_test(${NAME} ${NAME})
ENDMACRO()

MC_RTC_TEST(testConfiguration mc_control)

add_executable(test_controller_ticker test_controller_ticker.cpp)
pkg_config_use_dependency(test_controller_ticker mc_rbdyn_urdf_cpp)
target_link_libraries(test_controller_ticker ${Boost_LIBRARIES} mc_control)

add_subdirectory(controllers)
add_subdirectory(robots)
