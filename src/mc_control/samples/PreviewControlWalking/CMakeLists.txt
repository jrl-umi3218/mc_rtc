mc_rtc_set_prefix(STATES mc_controller/fsm/states)
mc_rtc_set_prefix(PreviewControlWalking_STATES mc_controller/PreviewControlWalking/states)

configure_file(etc/PreviewControlWalking.in.yaml ${CMAKE_CURRENT_BINARY_DIR}/etc/PreviewControlWalking.yaml)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/PreviewControlWalking.yaml
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/)

add_subdirectory(src)
