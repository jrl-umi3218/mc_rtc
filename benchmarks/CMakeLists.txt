#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

find_package(benchmark REQUIRED)

macro(mc_rtc_benchmark NAME)
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
  target_link_libraries(${NAME} benchmark::benchmark ${ARGN})
  generate_msvc_dot_user_file(${NAME})
  # Add benchmark as unit-test
  if(${BUILD_TESTING})
    add_test(${NAME} ${NAME})
  endif()
endmacro()

mc_rtc_benchmark(benchCompletionCriteria mc_control)
mc_rtc_benchmark(benchSimulationContactSensor mc_control)
mc_rtc_benchmark(benchRobotLoading mc_rbdyn)
mc_rtc_benchmark(benchAllocTasks mc_tasks)
