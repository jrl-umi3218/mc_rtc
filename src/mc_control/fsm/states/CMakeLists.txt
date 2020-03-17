#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(FSM_STATES_INSTALL_DIR "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(FSM_HDR_DIR "${PROJECT_SOURCE_DIR}/include/mc_control/fsm/states")
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

install(DIRECTORY data
  DESTINATION ${FSM_STATES_INSTALL_DIR}
  FILES_MATCHING 
  PATTERN "*.json"
  PATTERN "*.yaml")

macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${FSM_HDR_DIR}/${state_name}.h)
endmacro()

add_fsm_state_simple(HalfSitting)
add_fsm_state_simple(Grippers)
add_fsm_state_simple(Pause)
add_fsm_state_simple(Message)
add_fsm_state_simple(MetaTasks)
add_fsm_state_simple(AddRemoveContact)
add_fsm_state_simple(SlidingFootContact)
add_fsm_state_simple(Meta)
add_fsm_state_simple(Parallel)
add_fsm_state_simple(StabilizerStandingState)
target_link_libraries(StabilizerStandingState PUBLIC mc_planning)

# PythonState
if(${PYTHON_BINDING})
  get_cython_libraries(mc_rtc cython_libs)
  get_python_names(python_names)
  list(LENGTH python_names NSTATES)
  math(EXPR NSTATES "${NSTATES} - 1")
  foreach(i RANGE ${NSTATES})
    list(GET python_names ${i} PYTHON_NAME)
    list(GET cython_libs ${i} CYTHON_LIB)
    set(SRC "${CMAKE_CURRENT_BINARY_DIR}/generated/${PYTHON_NAME}State.cpp")
    configure_file(PythonState.cpp "${SRC}")
    add_library(${PYTHON_NAME}State SHARED "${SRC}")
    set_target_properties(${PYTHON_NAME}State PROPERTIES PREFIX "")
    set_target_properties(${PYTHON_NAME}State PROPERTIES FOLDER controllers/fsm/states)
    target_link_libraries(${PYTHON_NAME}State PUBLIC ${CYTHON_LIB} mc_control_fsm)
    install(TARGETS ${PYTHON_NAME}State DESTINATION ${FSM_STATES_INSTALL_DIR})
  endforeach()
endif()

install(
  EXPORT mc_rtc_fsm_states
  NAMESPACE mc_rtc::fsm::states::
  DESTINATION "${CONFIG_INSTALL_DIR}"
)
