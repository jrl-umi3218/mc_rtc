set(FSM_STATES_INSTALL_DIR "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(FSM_HDR_DIR "${PROJECT_SOURCE_DIR}/samples/mc_control/fsm/states")
set(FSM_HDR_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/mc_control/fsm/states")

macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED  ${state_SRC} ${state_HDR} "${PROJECT_SOURCE_DIR}/include/mc_control/fsm/states/api.h")
  set_target_properties(${state_name} PROPERTIES FOLDER controllers/fsm/states)
  set_target_properties(${state_name} PROPERTIES OUTPUT_NAME ${state_name})
  set_target_properties(${state_name} PROPERTIES PREFIX "")
  set_target_properties(${state_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS")
  target_link_libraries(${state_name} PUBLIC mc_control_fsm)
  install(TARGETS ${state_name} EXPORT mc_rtc_fsm_states DESTINATION ${FSM_STATES_INSTALL_DIR})
  install(FILES ${state_HDR} DESTINATION ${FSM_HDR_INSTALL_DIR})
endmacro()

macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${CMAKE_CURRENT_SOURCE_DIR}/${state_name}.h)
endmacro()

add_fsm_state_simple(CoMTrajectoryGeneration_Initial)
target_link_libraries(CoMTrajectoryGeneration_Initial PUBLIC mc_planning)

add_fsm_state_simple(StabilizerStandingTrackCoM)

# add_fsm_data_directory(data)
