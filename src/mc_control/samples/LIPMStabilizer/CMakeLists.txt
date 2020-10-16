add_subdirectory(src)

mc_rtc_set_prefix(STATES mc_controller/fsm/states)
set(LIPMStabilizer_STATES_INSTALL_PREFIX "${MC_STATES_RUNTIME_INSTALL_PREFIX}")
set(LIPMStabilizer_STATES_DATA_INSTALL_PREFIX "${MC_STATES_RUNTIME_INSTALL_PREFIX}/data")
set(LIPMStabilizer_INIT_STATE "Pause_2s")
configure_file(etc/LIPMStabilizer.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml")

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml"
  DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
