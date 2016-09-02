MACRO(CONTROLLER_TEST NAME)
  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} ${Boost_LIBRARIES} mc_control)
  set(TEST_CONTROLLER_NAME ${NAME})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/mc_rtc-${NAME}.conf)
  add_test(NAME ${NAME} COMMAND test_controller_ticker mc_rtc-${NAME}.conf)
ENDMACRO()

CONTROLLER_TEST(NotAController)
