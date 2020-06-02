add_subdirectory(src)

set(AROBASE "@")
set(LIPMStabilizer_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(LIPMStabilizer_STATES_DATA_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states/data")
set(LIPMStabilizer_INIT_STATE "Pause_2s")
configure_file(etc/LIPMStabilizer.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml")
unset(AROBASE)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml" 
  DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")
