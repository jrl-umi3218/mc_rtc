# -- Library install directory --
set(MC_RTC_LIBDIR "@CMAKE_INSTALL_PREFIX@/lib")

# -- Controllers --

set(MC_CONTROLLER_INSTALL_PREFIX "@CMAKE_INSTALL_PREFIX@/lib/mc_controller")

macro(add_controller controller_name controller_SRC controller_HDR)
  add_library(${controller_name} SHARED ${controller_SRC} ${controller_HDR})

  set_target_properties(${controller_name} PROPERTIES PREFIX "")
  if(DEFINED CATKIN_DEVEL_PREFIX)
    set_target_properties(${state_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/mc_controller")
  endif()

  target_link_libraries(${controller_name} PUBLIC mc_rtc::mc_control)

  install(TARGETS ${controller_name} DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}")
endmacro()

# -- Robots --

set(MC_ROBOTS_INSTALL_PREFIX "@CMAKE_INSTALL_PREFIX@/lib/mc_robots")

macro(add_robot robot_name robot_SRC robot_HDR)
  add_library(${robot_name} SHARED ${robot_SRC} ${robot_HDR})

  set_target_properties(${robot_name} PROPERTIES COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS" PREFIX "")

  target_link_libraries(${robot_name} PUBLIC mc_rtc::mc_rbdyn)

  install(TARGETS ${robot_base} DESTINATION "${MC_ROBOTS_INSTALL_PREFIX}")
endmacro()

macro(add_robot_simple robot_name)
  add_robot(${robot_name} ${robot_name}.cpp ${robot_name}.h)
endmacro()

# -- States --

set(MC_STATES_DEFAULT_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(MC_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/${PROJECT_NAME}/states")

macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED ${state_SRC} ${state_HDR})

  set_target_properties(${state_name} PROPERTIES PREFIX "")
  set_target_properties(${state_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS")
  if(DEFINED CATKIN_DEVEL_PREFIX)
    set_target_properties(${state_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/states")
  endif()

  target_link_libraries(${state_name} PUBLIC mc_rtc::mc_control_fsm)
  if(TARGET ${PROJECT_NAME})
    target_link_libraries(${state_name} PUBLIC ${PROJECT_NAME})
  endif()

  install(TARGETS ${state_name} DESTINATION "${MC_STATES_INSTALL_PREFIX}")
endmacro()

macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${state_name}.h)
endmacro()

macro(add_fsm_data_directory directory)
  install(DIRECTORY ${directory} DESTINATION "${MC_STATES_INSTALL_PREFIX}" FILES_MATCHING PATTERN "*.json")
endmacro()
