#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

macro(add_fsm_test_state state_name)
  set(state_SRC ${state_name}.cpp)
  if(EXISTS ${state_name}.h)
    set(state_SRC ${state_SRC} ${state_name}.h)
  endif()
  add_library(${state_name} SHARED ${state_SRC})
  set_target_properties(${state_name} PROPERTIES FOLDER tests/fsm/states)
  set_target_properties(${state_name} PROPERTIES
    COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS"
    PREFIX ""
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${state_name}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${state_name}
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${state_name})
  target_link_libraries(${state_name} PUBLIC mc_control_fsm)
endmacro()

add_fsm_test_state(SingleState)
add_fsm_test_state(MultipleStates)
add_fsm_test_state(ConfigureState)
