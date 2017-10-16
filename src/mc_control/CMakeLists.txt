macro(add_controller controller_name controller_SRC controller_HDR)
  add_library(${controller_name} SHARED ${controller_SRC} ${controller_HDR})
  set_target_properties(${controller_name} PROPERTIES COMPILE_FLAGS
    "-DMC_CONTROL_EXPORTS" PREFIX "")
  target_link_libraries(${controller_name} mc_control)
  pkg_config_use_dependency(${controller_name} mc_rbdyn_urdf)
  pkg_config_use_dependency(${controller_name} Tasks)
  target_link_libraries(${controller_name} ${Boost_LIBRARIES})
  install(TARGETS ${controller_name} DESTINATION
    ${MC_CONTROLLER_INSTALL_PREFIX})
endmacro()

if(${BUILD_CONTROLLER_SAMPLES})
  add_subdirectory(samples/Posture)
  add_subdirectory(samples/Body6d)
  add_subdirectory(samples/CoM)
  add_subdirectory(samples/Seq)
  add_subdirectory(samples/Driving)
  add_subdirectory(samples/Egress)
  add_subdirectory(samples/EgressMRQP)
  add_subdirectory(samples/BCISelfInteract)
  add_subdirectory(samples/Text)
endif()

file(GLOB EXT_CONTROLLERS RELATIVE ext "ext/*")
foreach(controller ${EXT_CONTROLLERS})
  if(EXISTS ext/${controller}/CMakeLists.txt)
    add_subdirectory(ext/${controller})
  endif()
endforeach()

add_controller("halfsitpose" "mc_halfsitpose_controller.cpp"
  "mc_halfsitpose_controller.h")

set(fsm_controller_SRC
  mc_fsm_controller.cpp
  mc_fsm_state_factory.cpp
)

set(fsm_controller_HDR
  ../../include/mc_control/mc_fsm_controller.h
  ../../include/mc_control/mc_fsm_state_factory.h
  ../../include/mc_control/mc_fsm_state.h
)

add_controller("fsm" "${fsm_controller_SRC}" "${fsm_controller_HDR}")

set(AROBASE "@")
configure_file(etc/FSM.conf.cmake ${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf)
unset(AROBASE)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/FSM.conf
  DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/etc/)

add_subdirectory(fsm_states)

if(${PYTHON_BINDING})
  add_subdirectory(python)
endif()
