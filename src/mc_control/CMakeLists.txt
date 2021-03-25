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

macro(add_fsm_controller controller_name controller_SRC controller_HDR)
  if(MC_RTC_BUILD_STATIC)
    target_sources(mc_control_fsm PRIVATE ${controller_SRC} ${controller_HDR} ${CMAKE_CURRENT_SOURCE_DIR}/lib.cpp)
  else()
    add_library(${controller_name} SHARED ${controller_SRC} ${controller_HDR})
    set_target_properties(${controller_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
    target_link_libraries(${controller_name} PUBLIC mc_control_fsm)
    install(TARGETS ${controller_name}
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    add_controller(${controller_name}_controller ${CMAKE_CURRENT_SOURCE_DIR}/lib.cpp "")
    set_target_properties(${controller_name}_controller PROPERTIES FOLDER controllers/fsm)
    target_link_libraries(${controller_name}_controller PUBLIC mc_control_fsm)
    target_link_libraries(${controller_name}_controller PUBLIC ${controller_name})
  endif()
endmacro()

add_subdirectory(fsm)

if(${BUILD_CONTROLLER_SAMPLES})
  add_subdirectory(samples/Posture)
  add_subdirectory(samples/EndEffector)
  add_subdirectory(samples/CoM)
  add_subdirectory(samples/Text)
  add_subdirectory(samples/FSM)
  add_subdirectory(samples/Door)
  add_subdirectory(samples/LIPMStabilizer)
  add_subdirectory(samples/Admittance)
  add_subdirectory(samples/Impedance)
  add_subdirectory(samples/ExternalForces)
endif()

add_subdirectory(HalfSitPose)

if(${PYTHON_BINDING})
  add_subdirectory(python)
endif()
