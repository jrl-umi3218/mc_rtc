macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED ${state_SRC} ${state_HDR})

  set_target_properties(${state_name} PROPERTIES PREFIX "")
  set_target_properties(${state_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS")
  set_target_properties(${state_name} PROPERTIES INSTALL_RPATH "${MC_STATES_DEFAULT_INSTALL_PREFIX};${MC_STATES_INSTALL_PREFIX};${Admittance_STATES_INSTALL_PREFIX}")

  target_link_libraries(${state_name} PUBLIC mc_rtc::mc_control_fsm)
  if(TARGET ${PROJECT_NAME})
    target_link_libraries(${state_name} PUBLIC ${PROJECT_NAME})
  endif()

  install(TARGETS ${state_name} DESTINATION "${Admittance_STATES_INSTALL_PREFIX}")
endmacro()

macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${state_name}.h)
endmacro()


add_fsm_state_simple(UpdateWall)
