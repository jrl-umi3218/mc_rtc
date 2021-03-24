macro(add_fsm_state state_name state_SRC state_HDR)
  if(MC_RTC_BUILD_STATIC)
    target_sources(mc_control_fsm PRIVATE ${state_SRC} ${state_HDR})
  else()
    add_library(${state_name} SHARED ${state_SRC} ${state_HDR})

    set_target_properties(${state_name} PROPERTIES PREFIX "")
    set_target_properties(${state_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS")
    set_target_properties(${state_name} PROPERTIES INSTALL_RPATH "${MC_STATES_RUNTIME_INSTALL_PREFIX};${MC_ExternalForces_STATES_RUNTIME_INSTALL_PREFIX}")

    target_link_libraries(${state_name} PUBLIC mc_rtc::mc_control_fsm external_forces)

    install(TARGETS ${state_name}
      ARCHIVE DESTINATION "${MC_ExternalForces_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
      LIBRARY DESTINATION "${MC_ExternalForces_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
      RUNTIME DESTINATION "${MC_ExternalForces_STATES_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
    )
  endif()
endmacro()

macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${CMAKE_CURRENT_SOURCE_DIR}/${state_name}.cpp ${CMAKE_CURRENT_SOURCE_DIR}/${state_name}.h)
endmacro()


add_fsm_state_simple(UpdateForces)
