get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

# -- Library install directory --
set(MC_RTC_LIBDIR "${PACKAGE_PREFIX_DIR}/lib")

# -- Controllers --

set(MC_CONTROLLER_INSTALL_PREFIX "${MC_RTC_LIBDIR}/mc_controller")

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

set(MC_ROBOTS_INSTALL_PREFIX "${MC_RTC_LIBDIR}/mc_robots")

macro(add_robot robot_name robot_SRC robot_HDR)
  add_library(${robot_name} SHARED ${robot_SRC} ${robot_HDR})

  set_target_properties(${robot_name} PROPERTIES COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS" PREFIX "")

  target_link_libraries(${robot_name} PUBLIC mc_rtc::mc_rbdyn)

  install(TARGETS ${robot_name} DESTINATION "${MC_ROBOTS_INSTALL_PREFIX}")
endmacro()

macro(add_robot_simple robot_name)
  add_robot(${robot_name} ${robot_name}.cpp ${robot_name}.h)
endmacro()

set(MC_ROBOTS_ALIASES_DIRECTORY "${MC_ROBOTS_INSTALL_PREFIX}/aliases/")
if(WIN32)
  set(MC_ROBOTS_USER_ALIASES_DIRECTORY "$ENV{APPDATA}/mc_rtc/aliases/")
else()
  set(MC_ROBOTS_USER_ALIASES_DIRECTORY "$ENV{HOME}/.config/mc_rtc/aliases/")
endif()

# -- Observers --
set(MC_OBSERVERS_INSTALL_PREFIX "${MC_RTC_LIBDIR}/mc_observers")

macro(add_observer observer_name observer_SRC observer_HDR)
  add_library(${observer_name} SHARED ${observer_SRC} ${observer_HDR})
  set_target_properties(${observer_name} PROPERTIES COMPILE_FLAGS "-DMC_OBSERVER_EXPORTS" PREFIX "")
  target_link_libraries(${observer_name} PUBLIC mc_rbdyn mc_rtc_gui)
  set_target_properties(${observer_name} PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_INSTALL_PREFIX})
  install(TARGETS ${observer_name} DESTINATION "${MC_OBSERVERS_INSTALL_PREFIX}")
endmacro()

macro(add_observer_simple observer_base)
  add_observer(${observer_base} ${observer_base}.cpp ${observer_base}.h)
endmacro()


# -- States --

set(MC_STATES_DEFAULT_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/fsm/states")
set(MC_STATES_INSTALL_PREFIX "${MC_CONTROLLER_INSTALL_PREFIX}/${PROJECT_NAME}/states")

macro(add_fsm_state state_name state_SRC state_HDR)
  add_library(${state_name} SHARED ${state_SRC} ${state_HDR})

  set_target_properties(${state_name} PROPERTIES PREFIX "")
  set_target_properties(${state_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS")
  set_target_properties(${state_name} PROPERTIES INSTALL_RPATH "${MC_STATES_DEFAULT_INSTALL_PREFIX};${MC_STATES_INSTALL_PREFIX}")
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
  install(DIRECTORY ${directory} DESTINATION "${MC_STATES_INSTALL_PREFIX}" FILES_MATCHING REGEX ".*(.json$|.yml$|.yaml$)")
endmacro()

# -- Helper to find a description package --

macro(find_description_package PACKAGE)
  set(PACKAGE_PATH_VAR "${PACKAGE}_PATH")
  string(TOUPPER "${PACKAGE_PATH_VAR}" PACKAGE_PATH_VAR)
  find_package(${PACKAGE} REQUIRED)
  if("${${PACKAGE}_INSTALL_PREFIX}" STREQUAL "")
    if("${${PACKAGE}_SOURCE_PREFIX}" STREQUAL "")
      message(FATAL_ERROR "Your ${PACKAGE} does not define where to find the data, please update.")
    else()
      set(${PACKAGE_PATH_VAR} "${${PACKAGE}_SOURCE_PREFIX}")
    endif()
  else()
    set(${PACKAGE_PATH_VAR} "${${PACKAGE}_INSTALL_PREFIX}")
  endif()
  # Cleanup the path provided by CMake
  get_filename_component(${PACKAGE_PATH_VAR} "${${PACKAGE_PATH_VAR}}" REALPATH)
  message("-- Found ${PACKAGE}: ${${PACKAGE_PATH_VAR}}")
endmacro()

# -- Helper to create a new mc_rtc plugin

set(MC_PLUGINS_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/lib/mc_plugins)

macro(add_plugin plugin plugin_SRC plugin_HDR)
  option(AUTOLOAD_${plugin}_PLUGIN "Automatically load ${plugin} plugin" ON)
  add_library(${plugin} SHARED ${plugin_SRC} ${plugin_HDR})
  set_target_properties(${plugin} PROPERTIES PREFIX "")
  target_link_libraries(${plugin} PUBLIC mc_rtc::mc_control)
  install(TARGETS ${plugin} DESTINATION ${MC_PLUGINS_INSTALL_PREFIX})
  set(plugin_CFG "${CMAKE_CURRENT_SOURCE_DIR}/etc/${plugin}.yaml")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/autoload/${plugin}.yaml" "${plugin}")
  if(AUTOLOAD_${plugin}_PLUGIN)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/autoload/${plugin}.yaml"
      DESTINATION "${MC_PLUGINS_INSTALL_PREFIX}/autoload/")
  endif()
endmacro()

file(GLOB plugin_targets "${CMAKE_CURRENT_LIST_DIR}/plugins/*Targets.cmake")
foreach(plugin ${plugin_targets})
  include(${plugin})
endforeach()
