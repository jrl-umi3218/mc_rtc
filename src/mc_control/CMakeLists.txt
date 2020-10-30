#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

macro(add_controller controller_name controller_SRC controller_HDR)
  if(MC_RTC_BUILD_STATIC)
    target_sources(mc_control PRIVATE ${controller_SRC} ${controller_HDR})
  else()
    add_library(${controller_name} SHARED ${controller_SRC} ${controller_HDR})
    set_target_properties(${controller_name} PROPERTIES FOLDER controllers)
    set_target_properties(${controller_name} PROPERTIES COMPILE_FLAGS
      "-DMC_CONTROL_EXPORTS" PREFIX "")
    target_link_libraries(${controller_name} PUBLIC mc_control)
    install(TARGETS ${controller_name}
      ARCHIVE DESTINATION "${MC_CONTROLLER_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
      LIBRARY DESTINATION "${MC_CONTROLLER_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
      RUNTIME DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
    )
  endif()
endmacro()

if(${BUILD_CONTROLLER_SAMPLES})
  add_subdirectory(samples/Posture)
  add_subdirectory(samples/Body6d)
  add_subdirectory(samples/CoM)
  add_subdirectory(samples/Text)
  # FIXME Get simple controllers working first
  #add_subdirectory(samples/FSM)
  #add_subdirectory(samples/LIPMStabilizer)
  #add_subdirectory(samples/Admittance)
  #add_subdirectory(samples/Door)
endif()

add_subdirectory(HalfSitPose)
add_subdirectory(fsm)

if(${PYTHON_BINDING})
  add_subdirectory(python)
endif()
