#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

macro(add_controller controller_name controller_SRC controller_HDR)
  add_library(${controller_name} SHARED ${controller_SRC} ${controller_HDR})
  set_target_properties(${controller_name} PROPERTIES FOLDER controllers)
  set_target_properties(${controller_name} PROPERTIES COMPILE_FLAGS
    "-DMC_CONTROL_EXPORTS" PREFIX "")
  target_link_libraries(${controller_name} PUBLIC mc_control)
  install(TARGETS ${controller_name} DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX})
endmacro()

if(${BUILD_CONTROLLER_SAMPLES})
  add_subdirectory(samples/Posture)
  add_subdirectory(samples/Body6d)
  add_subdirectory(samples/CoM)
  add_subdirectory(samples/Text)
  add_subdirectory(samples/LIPMStabilizer)
endif()

add_subdirectory(HalfSitPose)
add_subdirectory(fsm)

if(${PYTHON_BINDING})
  add_subdirectory(python)
endif()
