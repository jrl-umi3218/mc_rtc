set(ROBOT_MODULE_PATH "${CMAKE_BINARY_DIR}/src/mc_robots")
if(CMAKE_CONFIGURATION_TYPES)
  set(ROBOT_MODULE_PATH "${ROBOT_MODULE_PATH}/$<CONFIG>")
endif()

function(make_configuration VAR_OUT MAIN_ROBOT)
  set(CFG_CMAKE_OUT "${CMAKE_CURRENT_BINARY_DIR}/${VAR_OUT}/etc/mc_rtc.cmake.yaml")
  set(CFG_OUT "${CMAKE_CURRENT_BINARY_DIR}/${VAR_OUT}/etc/$<CONFIG>/mc_rtc.yaml")
  configure_file(mc_rtc.in.yaml "${CFG_CMAKE_OUT}" @ONLY)
  file(
    GENERATE
    OUTPUT "${CFG_OUT}"
    INPUT "${CFG_CMAKE_OUT}"
  )
  set(${VAR_OUT}
      "${CFG_OUT}"
      PARENT_SCOPE
  )
endfunction()

make_configuration(MAIN_ROBOT_SIMPLE_CFG "JVRC1")
make_configuration(MAIN_ROBOT_SIMPLE_VECTOR_CFG "[JVRC1]")
make_configuration(MAIN_ROBOT_OBJECT_CFG "{module: JVRC1, name: JVRC1}")
make_configuration(MAIN_ROBOT_OBJECT_VECTOR_CFG "{module: [JVRC1], name: JVRC1}")

set(CPP_CMAKE_OUT
    "${CMAKE_CURRENT_BINARY_DIR}/testGlobalControllerConfiguration.cmake.cpp"
)
set(CPP_OUT
    "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/testGlobalControllerConfiguration.cpp"
)
configure_file(testGlobalControllerConfiguration.in.cpp "${CPP_CMAKE_OUT}" @ONLY)
file(
  GENERATE
  OUTPUT "${CPP_OUT}"
  INPUT "${CPP_CMAKE_OUT}"
)

add_executable(testGlobalControllerConfiguration "${CPP_OUT}")
target_link_libraries(
  testGlobalControllerConfiguration
  PUBLIC mc_rtc::mc_control Boost::unit_test_framework Boost::disable_autolinking
)
if(NOT Boost_USE_STATIC_LIBS)
  target_link_libraries(testGlobalControllerConfiguration PUBLIC Boost::dynamic_linking)
endif()
target_compile_definitions(
  testGlobalControllerConfiguration PRIVATE -DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN
)
set_target_properties(
  testGlobalControllerConfiguration PROPERTIES FOLDER tests/framework
)
add_test(NAME testGlobalControllerConfiguration
         COMMAND testGlobalControllerConfiguration
)
generate_msvc_dot_user_file(testGlobalControllerConfiguration)
