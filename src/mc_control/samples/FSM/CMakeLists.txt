add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/FSM.conf.cmake ${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf)
unset(AROBASE)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf
  DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/etc/)
