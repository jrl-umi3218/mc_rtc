set(MC_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(Door_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/Door/states")
set(Door_STATES_DATA_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/Door/states/data")
configure_file(etc/DoorSample.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/DoorSample.yaml)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/DoorSample.yaml
  DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/etc/)

add_subdirectory(src)
