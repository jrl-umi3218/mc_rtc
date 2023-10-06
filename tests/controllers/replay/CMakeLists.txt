configure_file(mc_rtc.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/mc_rtc.in.yaml)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/logs)
file(
  GENERATE
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/mc_rtc.yaml
  INPUT ${CMAKE_CURRENT_BINARY_DIR}/mc_rtc.in.yaml
)
configure_file(
  TestReplayController_Play.in.yaml
  ${CMAKE_CURRENT_BINARY_DIR}/TestReplayController_Play.in.yaml
)
file(
  GENERATE
  OUTPUT $<TARGET_FILE_DIR:TestReplayController>/etc/TestReplayController_Play.yaml
  INPUT ${CMAKE_CURRENT_BINARY_DIR}/TestReplayController_Play.in.yaml
)

add_library(TestReplayController TestReplayController.cpp)
target_link_libraries(TestReplayController PUBLIC mc_control Boost::unit_test_framework)
if(NOT Boost_USE_STATIC_LIBS)
  target_link_libraries(TestReplayController PUBLIC Boost::dynamic_linking)
endif()
set_target_properties(
  TestReplayController
  PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS"
             PREFIX ""
             ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestReplayController
             LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestReplayController
             RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/TestReplayController
)
target_include_directories(TestReplayController PRIVATE ${CMAKE_SOURCE_DIR}/tests)
set(CONFIG_HEADER_INCLUDE_DIR "${CMAKE_BINARY_DIR}/tests/include/$<CONFIG>")
target_include_directories(TestReplayController PRIVATE ${CONFIG_HEADER_INCLUDE_DIR})

add_global_controller_test_run(
  TestReplayController_Record ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/mc_rtc.yaml 50
  TestReplayController_Play
)
