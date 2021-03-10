mc_rtc_set_prefix(STATES mc_controller/fsm/states)
mc_rtc_set_prefix(ExternalForces_STATES mc_controller/ExternalForces/states)

configure_file(etc/ExternalForces.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/ExternalForces.yaml)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/ExternalForces.yaml
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/)

add_subdirectory(src)
