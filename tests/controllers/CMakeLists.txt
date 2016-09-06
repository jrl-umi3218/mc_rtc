macro(CONTROLLER_TEST_COMMON NAME)
  add_library(${NAME} SHARED ${NAME}.cpp)
  target_link_libraries(${NAME} ${Boost_LIBRARIES} mc_control)
  set_target_properties(${NAME} PROPERTIES COMPILE_FLAGS
    "-DMC_CONTROL_EXPORTS" PREFIX "")
  set(TEST_CONTROLLER_NAME ${NAME})
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/mc_rtc.conf.in ${CMAKE_CURRENT_BINARY_DIR}/mc_rtc-${NAME}.conf)
endmacro()

macro(CONTROLLER_TEST_CONSTRUCTION_FAILURE NAME)
  CONTROLLER_TEST_COMMON(${NAME})
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=CONSTRUCTION_FAILURE mc_rtc-${NAME}.conf)
endmacro()

macro(CONTROLLER_TEST_RUN NAME NRITER)
  CONTROLLER_TEST_COMMON(${NAME})
  add_test(NAME ${NAME} COMMAND test_controller_ticker --run_test=RUN mc_rtc-${NAME}.conf ${NRITER})
endmacro()

CONTROLLER_TEST_CONSTRUCTION_FAILURE(NotAController)

# To run properly this test requires controller creation sandboxing which is only enabled on Linux
if(UNIX AND NOT APPLE)
  CONTROLLER_TEST_CONSTRUCTION_FAILURE(UnresolvedSymbolController)
endif()
