find_package(benchmark REQUIRED)

macro(mc_rtc_benchmark NAME)
  add_executable(${NAME} ${NAME}.cpp)
  target_link_libraries(${NAME} benchmark ${ARGN})
  pkg_config_use_dependency(${NAME} Tasks)
  generate_msvc_dot_user_file(${NAME})
endmacro()

mc_rtc_benchmark(benchCompletionCriteria mc_control)
