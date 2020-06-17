add_subdirectory(src)

set(AROBASE "@")
set(CoMTrajectoryGeneration_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(CoMTrajectoryGeneration_STATES_DATA_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states/data")
set(CoMTrajectoryGeneration_INIT_STATE "Pause_2s")
set(MC_STATES_DEFAULT_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(MC_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/${PROJECT_NAME}/states")
configure_file(etc/CoMTrajectoryGeneration.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/CoMTrajectoryGeneration.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/CoMTrajectoryGeneration.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")
unset(AROBASE)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/CoMTrajectoryGeneration.yaml" 
  DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")
