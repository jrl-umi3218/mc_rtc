add_subdirectory(src)

mc_rtc_set_prefix(STATES mc_controller/fsm/states)
set(LIPMStabilizer_STATES_DESTINATION_PREFIX "${MC_STATES_RUNTIME_DESTINATION_PREFIX}")
set(LIPMStabilizer_STATES_DATA_DESTINATION_PREFIX
    "${MC_STATES_RUNTIME_DESTINATION_PREFIX}/data"
)
set(LIPMStabilizer_INIT_STATE "LIPMStabilizer::Pause_2s")
configure_file(
  etc/LIPMStabilizer.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml"
)
configure_file(
  etc/LIPMStabilizer.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer_TVM.yaml"
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer.yaml"
              "${CMAKE_CURRENT_BINARY_DIR}/etc/LIPMStabilizer_TVM.yaml"
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc"
)
