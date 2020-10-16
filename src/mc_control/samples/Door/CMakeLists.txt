mc_rtc_set_prefix(STATES mc_controller/fsm/states)
mc_rtc_set_prefix(Door_STATES mc_controller/Door/states)
set(MC_Door_STATES_DATA_INSTALL_PREFIX "${MC_Door_STATES_RUNTIME_INSTALL_PREFIX}/data")
configure_file(etc/DoorSample.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/DoorSample.yaml)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/DoorSample.yaml
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/)

add_subdirectory(src)
