set(MC_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(Admittance_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/AdmittanceSample/states")
set(Admittance_STATES_DATA_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/AdmittanceSample/states/data")
configure_file(etc/AdmittanceSample.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/AdmittanceSample.yaml)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/AdmittanceSample.yaml
  DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/etc/)

add_subdirectory(src)
