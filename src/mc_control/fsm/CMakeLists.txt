set(fsm_controller_SRC
  Controller.cpp
  StateFactory.cpp
)

set(fsm_controller_HDR_DIR ${PROJECT_SOURCE_DIR}/include/mc_control/fsm)
set(fsm_controller_HDR
  ${fsm_controller_HDR_DIR}/Controller.h
  ${fsm_controller_HDR_DIR}/State.h
  ${fsm_controller_HDR_DIR}/StateFactory.h
)

add_controller("fsm" "${fsm_controller_SRC}" "${fsm_controller_HDR}")

set(AROBASE "@")
configure_file(etc/FSM.conf.cmake ${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf)
unset(AROBASE)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf
  DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/etc/)

add_subdirectory(states)
