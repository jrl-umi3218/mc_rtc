macro(add_controller controller_name controller_SRC controller_HDR)
  add_library(${controller_name} SHARED ${controller_SRC} ${controller_HDR})
  set_target_properties(${controller_name} PROPERTIES COMPILE_FLAGS
    "-DMC_CONTROL_EXPORTS" PREFIX "")
  target_link_libraries(${controller_name} mc_control)
  pkg_config_use_dependency(${controller_name} mc_rbdyn_urdf_cpp)
  pkg_config_use_dependency(${controller_name} Tasks)
  target_link_libraries(${controller_name} ${Boost_LIBRARIES})
  install(TARGETS ${controller_name} DESTINATION
    ${MC_CONTROLLER_INSTALL_PREFIX})
endmacro()

if(${BUILD_CONTROLLER_SAMPLES})
  add_controller("posture" "mc_posture_controller.cpp"
    "mc_posture_controller.h")
  add_controller("body6d" "mc_body6d_controller.cpp"
    "mc_body6d_controller.h")
  add_controller("com" "mc_com_controller.cpp"
    "mc_com_controller.h")
  add_controller("seq"
    "mc_seq_controller.cpp;mc_seq_steps.cpp;MCSeqPublisher.cpp"
    "mc_seq_controller.h;mc_seq_actions.h;mc_seq_steps.h;MCSeqPublisher.h;json/StanceConfig.h")
  add_controller("driving" "mc_driving_controller.cpp"
    "mc_driving_controller.h")
  add_controller("egress" "mc_egress_controller.cpp"
    "mc_egress_controller.h")
  add_controller("egress_mrqp" "mc_egress_mrqp_controller.cpp"
    "mc_egress_mrqp_controller.h")
  add_controller("bci_self_interact"
    "mc_bci_self_interact_controller.cpp"
    "mc_bci_self_interact_controller.h")
endif()

add_controller("halfsitpose" "mc_halfsitpose_controller.cpp"
  "mc_halfsitpose_controller.h")
