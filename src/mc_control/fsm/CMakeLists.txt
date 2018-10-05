set(fsm_controller_SRC
  Controller.cpp
  Executor.cpp
  State.cpp
  StateFactory.cpp
  TransitionMap.cpp
)

set(fsm_controller_HDR_DIR ${PROJECT_SOURCE_DIR}/include/mc_control/fsm)
set(fsm_controller_HDR
  ${fsm_controller_HDR_DIR}/api.h
  ${fsm_controller_HDR_DIR}/Controller.h
  ${fsm_controller_HDR_DIR}/Executor.h
  ${fsm_controller_HDR_DIR}/State.h
  ${fsm_controller_HDR_DIR}/StateFactory.h
  ${fsm_controller_HDR_DIR}/Transition.h
  ${fsm_controller_HDR_DIR}/TransitionMap.h
)

if(${PYTHON_BINDING})
  set(fsm_controller_HDR ${fsm_controller_HDR} ${fsm_controller_HDR_DIR}/PythonState.h)
  set(fsm_controller_SRC ${fsm_controller_SRC} PythonState.cpp)
endif()

add_library(mc_control_fsm SHARED ${fsm_controller_SRC} ${fsm_controller_HDR})
set_target_properties(mc_control_fsm PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_EXPORTS")
target_link_libraries(mc_control_fsm PUBLIC mc_control ${Boost_LIBRARIES})
pkg_config_use_dependency(mc_control_fsm mc_rbdyn_urdf)
pkg_config_use_dependency(mc_control_fsm Tasks)
if(GENERATE_COVERAGE)
  target_compile_options(mc_control_fsm PRIVATE "--coverage")
  set_target_properties(mc_control_fsm PROPERTIES LINK_FLAGS "--coverage")
endif()
set_target_properties(mc_control_fsm PROPERTIES SOVERSION 0 VERSION 0.1.0)
install(TARGETS mc_control_fsm
  EXPORT "${TARGETS_EXPORT_NAME}"
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

add_controller("fsm" "lib.cpp" "")
target_link_libraries(fsm PUBLIC mc_control_fsm)

set(AROBASE "@")
configure_file(etc/FSM.conf.cmake ${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf)
unset(AROBASE)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf
  DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/etc/)

add_subdirectory(states)
