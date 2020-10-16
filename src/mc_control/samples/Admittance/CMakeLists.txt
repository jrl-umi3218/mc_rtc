mc_rtc_set_prefix(STATES mc_controller/fsm/states)
mc_rtc_set_prefix(Admittance_STATES mc_controller/AdmittanceSample/states)

configure_file(etc/AdmittanceSample.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/AdmittanceSample.yaml)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/AdmittanceSample.yaml
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/)

add_subdirectory(src)
