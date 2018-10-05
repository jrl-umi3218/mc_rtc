set(FSM_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")

macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED  ${state_SRC} ${state_HDR} "${PROJECT_SOURCE_DIR}/include/mc_control/fsm/states/api.h")
  set_target_properties(${state_name} PROPERTIES PREFIX "")
  set_target_properties(${state_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS")
  target_link_libraries(${state_name} PUBLIC mc_control_fsm)
  install(TARGETS ${state_name}
    DESTINATION ${FSM_STATES_INSTALL_PREFIX})
  install(FILES ${state_HDR}
          DESTINATION ${CMAKE_INSTALL_PREFIX}/include/mc_control/fsm/states)
endmacro()

install(DIRECTORY data
  DESTINATION ${FSM_STATES_INSTALL_PREFIX}
  FILES_MATCHING PATTERN "*.json")

macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${PROJECT_SOURCE_DIR}/include/mc_control/fsm/states/${state_name}.h)
endmacro()

add_fsm_state_simple(HalfSitting)
add_fsm_state_simple(Grippers)
add_fsm_state_simple(Pause)
add_fsm_state_simple(MetaTasks)
add_fsm_state_simple(AddRemoveContact)
add_fsm_state_simple(SlidingFootContact)
add_fsm_state_simple(Meta)
add_fsm_state_simple(Parallel)

# PythonState
if(${PYTHON_BINDING})
  include(${PROJECT_SOURCE_DIR}/cmake/python.cmake)
  findpython()
  add_library(PythonState SHARED "PythonState.cpp")
  set_target_properties(PythonState PROPERTIES PREFIX "")
  target_include_directories("PythonState" PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/../../../../binding/python/$<CONFIGURATION>" ${PYTHON_INCLUDE_DIRS})
  target_link_libraries(PythonState PUBLIC mc_control_fsm ${PYTHON_LIBRARIES})
  add_dependencies(PythonState mc_rtc-python-bindings)
  install(TARGETS PythonState
    DESTINATION ${FSM_STATES_INSTALL_PREFIX})
endif()
