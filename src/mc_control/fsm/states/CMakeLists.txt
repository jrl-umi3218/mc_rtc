#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

mc_rtc_set_prefix(FSM_STATES mc_controller/fsm/states)
set(FSM_HDR_DIR "${PROJECT_SOURCE_DIR}/include/mc_control/fsm/states")
set(FSM_HDR_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include/mc_control/fsm/states")

macro(add_fsm_state state_name state_SRC state_HDR)
  if(MC_RTC_BUILD_STATIC)
    target_sources(mc_control_fsm PRIVATE ${state_SRC} ${state_HDR})
  else()
    add_library(
      ${state_name} SHARED ${state_SRC} ${state_HDR}
                           "${MC_RTC_SOURCE_DIR}/include/mc_control/fsm/states/api.h"
    )
    set_target_properties(${state_name} PROPERTIES FOLDER controllers/fsm/states)
    set_target_properties(${state_name} PROPERTIES OUTPUT_NAME ${state_name})
    set_target_properties(${state_name} PROPERTIES PREFIX "")
    set_target_properties(
      ${state_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS"
    )
    target_link_libraries(${state_name} PUBLIC mc_control_fsm)
    install(
      TARGETS ${state_name}
      EXPORT mc_rtc_fsm_states
      ARCHIVE
        DESTINATION
          "${MC_FSM_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
      LIBRARY
        DESTINATION
          "${MC_FSM_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
      RUNTIME
        DESTINATION
          "${MC_FSM_STATES_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    )
  endif()
  install(FILES ${state_HDR} DESTINATION ${FSM_HDR_INSTALL_DIR})
endmacro()

install(
  DIRECTORY data
  DESTINATION ${MC_FSM_STATES_RUNTIME_INSTALL_PREFIX}
  FILES_MATCHING
  PATTERN "*.json"
  PATTERN "*.yaml"
)

macro(add_fsm_state_simple state_name)
  add_fsm_state(
    ${state_name} ${CMAKE_CURRENT_SOURCE_DIR}/${state_name}.cpp
    ${FSM_HDR_DIR}/${state_name}.h
  )
endmacro()

add_fsm_state_simple(HalfSitting)
add_fsm_state_simple(Grippers)
add_fsm_state_simple(Pause)
add_fsm_state_simple(Posture)
add_fsm_state_simple(Message)
add_fsm_state_simple(MetaTasks)
add_fsm_state_simple(AddRemoveContact)
add_fsm_state_simple(Meta)
add_fsm_state_simple(Parallel)
add_fsm_state_simple(StabilizerStandingState)
add_fsm_state_simple(EnableController)

# PythonState
if(${PYTHON_BINDING})
  foreach(PYTHON_NAME ${PYTHON_BINDING_VERSIONS})
    set(SRC "${CMAKE_CURRENT_BINARY_DIR}/generated/${PYTHON_NAME}State.cpp")
    configure_file(PythonState.cpp "${SRC}")
    add_library(${PYTHON_NAME}State SHARED "${SRC}")
    set_target_properties(${PYTHON_NAME}State PROPERTIES PREFIX "")
    set_target_properties(${PYTHON_NAME}State PROPERTIES FOLDER controllers/fsm/states)
    target_link_libraries(
      ${PYTHON_NAME}State PRIVATE ${PYTHON_NAME}::Python mc_control_fsm
    )
    target_include_directories(${PYTHON_NAME}State PRIVATE "${PROJECT_SOURCE_DIR}/src")
    add_dependencies(
      ${PYTHON_NAME}State mc_control_fsm_${PYTHON_NAME} mc_control_${PYTHON_NAME}
      mc_rbdyn_${PYTHON_NAME}
    )
    target_include_directories(
      ${PYTHON_NAME}State
      PRIVATE
        $<TARGET_PROPERTY:mc_control_fsm_${PYTHON_NAME},INTERFACE_INCLUDE_DIRECTORIES>
    )
    target_compile_definitions(
      ${PYTHON_NAME}State
      PRIVATE
        $<TARGET_PROPERTY:mc_control_fsm_${PYTHON_NAME},INTERFACE_COMPILE_DEFINITIONS>
    )
    install(
      TARGETS ${PYTHON_NAME}State
      ARCHIVE
        DESTINATION
          "${MC_FSM_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
      LIBRARY
        DESTINATION
          "${MC_FSM_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
      RUNTIME
        DESTINATION
          "${MC_FSM_STATES_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    )
  endforeach()
endif()

if(NOT MC_RTC_BUILD_STATIC)
  install(
    EXPORT mc_rtc_fsm_states
    NAMESPACE mc_rtc::fsm::states::
    DESTINATION "${CONFIG_INSTALL_DIR}"
  )
endif()
