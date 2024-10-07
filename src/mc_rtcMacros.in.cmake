get_filename_component(
  PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/@mc_rtc_macros_RELATIVE_PATH@" ABSOLUTE
)

# -- Library install directory --
set(MC_RTC_BINDIR "${PACKAGE_PREFIX_DIR}/@CMAKE_INSTALL_BINDIR@")
set(MC_RTC_DOCDIR "${PACKAGE_PREFIX_DIR}/@CMAKE_INSTALL_DOCDIR@")
set(MC_RTC_LIBDIR "${PACKAGE_PREFIX_DIR}/@CMAKE_INSTALL_LIBDIR@")

# Allow to override mc_rtc debug suffix
if(NOT DEFINED MC_RTC_LOADER_DEBUG_SUFFIX)
  set(MC_RTC_LOADER_DEBUG_SUFFIX "@MC_RTC_LOADER_DEBUG_SUFFIX@")
endif()

if(NOT DEFINED MC_RTC_HONOR_INSTALL_PREFIX)
  set(MC_RTC_HONOR_INSTALL_PREFIX OFF)
endif()

if(MC_RTC_HONOR_INSTALL_PREFIX)
  include(GNUInstallDirs)
endif()

# -- Helper to set the components install prefix --
macro(mc_rtc_set_prefix NAME FOLDER)
  if(MC_RTC_HONOR_INSTALL_PREFIX)
    set(MC_${NAME}_LIBRARY_INSTALL_PREFIX "${CMAKE_INSTALL_FULL_LIBDIR}/${FOLDER}")
  else()
    set(MC_${NAME}_LIBRARY_INSTALL_PREFIX "${MC_RTC_LIBDIR}/${FOLDER}")
  endif()
  if(WIN32)
    if(MC_RTC_HONOR_INSTALL_PREFIX)
      set(MC_${NAME}_RUNTIME_INSTALL_PREFIX "${CMAKE_INSTALL_FULL_BINDIR}/${FOLDER}")
    else()
      set(MC_${NAME}_RUNTIME_INSTALL_PREFIX "${MC_RTC_BINDIR}/${FOLDER}")
    endif()
  else()
    set(MC_${NAME}_RUNTIME_INSTALL_PREFIX "${MC_${NAME}_LIBRARY_INSTALL_PREFIX}")
  endif()
  # For backward compatibility
  set(MC_${NAME}_INSTALL_PREFIX "${MC_${NAME}_LIBRARY_INSTALL_PREFIX}")
endmacro()

# -- Controllers --

mc_rtc_set_prefix(CONTROLLER mc_controller)

macro(add_controller controller_name)
  add_library(${controller_name} SHARED ${ARGN})

  set_target_properties(${controller_name} PROPERTIES PREFIX "")
  if(DEFINED CATKIN_DEVEL_PREFIX)
    set_target_properties(
      ${controller_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                    "${CATKIN_DEVEL_PREFIX}/lib/mc_controller"
    )
  endif()

  target_link_libraries(${controller_name} PUBLIC mc_rtc::mc_control)

  install(
    TARGETS ${controller_name}
    ARCHIVE
      DESTINATION
        "${MC_CONTROLLER_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    LIBRARY
      DESTINATION
        "${MC_CONTROLLER_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    RUNTIME
      DESTINATION
        "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
  )
endmacro()

macro(install_controller_configuration CONFIG)
  install(FILES "${CONFIG}" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
endmacro()

macro(install_controller_robot_configuration CONTROLLER_NAME CONFIG)
  install(FILES "${CONFIG}"
          DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/${CONTROLLER_NAME}/"
  )
endmacro()

# -- Robots --

mc_rtc_set_prefix(ROBOTS mc_robots)

macro(add_robot robot_name)
  add_library(${robot_name} SHARED ${ARGN})

  set_target_properties(
    ${robot_name} PROPERTIES COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS" PREFIX ""
  )

  target_link_libraries(${robot_name} PUBLIC mc_rtc::mc_rbdyn)

  install(
    TARGETS ${robot_name}
    ARCHIVE
      DESTINATION
        "${MC_ROBOTS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    LIBRARY
      DESTINATION
        "${MC_ROBOTS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    RUNTIME
      DESTINATION
        "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
  )
endmacro()

macro(add_robot_simple robot_name)
  add_robot(${robot_name} ${robot_name}.cpp ${robot_name}.h)
endmacro()

set(MC_ROBOTS_ALIASES_DIRECTORY "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}/aliases/")
if(WIN32)
  set(MC_ROBOTS_USER_ALIASES_DIRECTORY "$ENV{APPDATA}/mc_rtc/aliases/")
else()
  set(MC_ROBOTS_USER_ALIASES_DIRECTORY "$ENV{HOME}/.config/mc_rtc/aliases/")
endif()

# -- Observers --
mc_rtc_set_prefix(OBSERVERS mc_observers)

macro(add_observer observer_name)
  add_library(${observer_name} SHARED ${ARGN})
  set_target_properties(
    ${observer_name} PROPERTIES COMPILE_FLAGS "-DMC_OBSERVER_EXPORTS" PREFIX ""
  )
  set_target_properties(
    ${observer_name} PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX}
  )
  target_link_libraries(${observer_name} PUBLIC mc_rtc::mc_observers)
  install(
    TARGETS ${observer_name}
    ARCHIVE
      DESTINATION
        "${MC_OBSERVERS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    LIBRARY
      DESTINATION
        "${MC_OBSERVERS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    RUNTIME
      DESTINATION
        "${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
  )
endmacro()

macro(add_observer_simple observer_base)
  add_observer(${observer_base} ${observer_base}.cpp ${observer_base}.h)
endmacro()

macro(install_observer_configuration CONFIG)
  install(FILES "${CONFIG}" DESTINATION "${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX}/etc")
endmacro()

macro(install_observer_robot_configuration OBSERVER_NAME CONFIG)
  install(FILES "${CONFIG}"
          DESTINATION "${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX}/${OBSERVER_NAME}/"
  )
endmacro()

# -- For backward compatibilty we keep mc_observers as an alias to mc_control
add_library(mc_rtc::mc_observers INTERFACE IMPORTED)
set_target_properties(
  mc_rtc::mc_observers PROPERTIES INTERFACE_LINK_LIBRARIES mc_rtc::mc_control
)

# -- States --
if(MC_RTC_HONOR_INSTALL_PREFIX)
  set(MC_RTC_HONOR_INSTALL_PREFIX OFF)
  mc_rtc_set_prefix(STATES_DEFAULT mc_controller/fsm/states)
  set(MC_RTC_HONOR_INSTALL_PREFIX ON)
else()
  mc_rtc_set_prefix(STATES_DEFAULT mc_controller/fsm/states)
endif()
mc_rtc_set_prefix(STATES mc_controller/${PROJECT_NAME}/states)

macro(add_fsm_state state_name)
  add_library(${state_name} SHARED ${ARGN})

  set_target_properties(${state_name} PROPERTIES PREFIX "")
  set_target_properties(
    ${state_name} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_STATE_EXPORTS"
  )
  set_target_properties(
    ${state_name}
    PROPERTIES
      INSTALL_RPATH
      "${MC_STATES_DEFAULT_RUNTIME_INSTALL_PREFIX};${MC_STATES_RUNTIME_INSTALL_PREFIX}"
  )
  if(DEFINED CATKIN_DEVEL_PREFIX)
    set_target_properties(
      ${state_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                               "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}/states"
    )
  endif()

  target_link_libraries(${state_name} PUBLIC mc_rtc::mc_control_fsm)
  if(TARGET ${PROJECT_NAME})
    target_link_libraries(${state_name} PUBLIC ${PROJECT_NAME})
  endif()

  install(
    TARGETS ${state_name}
    ARCHIVE
      DESTINATION
        "${MC_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    LIBRARY
      DESTINATION
        "${MC_STATES_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    RUNTIME
      DESTINATION
        "${MC_STATES_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
  )
endmacro()

macro(add_fsm_state_simple state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${state_name}.h)
endmacro()

macro(add_fsm_data_directory directory)
  install(
    DIRECTORY ${directory}
    DESTINATION "${MC_STATES_RUNTIME_INSTALL_PREFIX}"
    FILES_MATCHING
    REGEX ".*(.json$|.yml$|.yaml$)"
  )
endmacro()

# -- Helper to find a description package --

macro(find_description_package PACKAGE)
  set(PACKAGE_PATH_VAR "${PACKAGE}_PATH")
  string(TOUPPER "${PACKAGE_PATH_VAR}" PACKAGE_PATH_VAR)
  find_package(${PACKAGE} REQUIRED)
  if("${${PACKAGE}_INSTALL_PREFIX}" STREQUAL "")
    if("${${PACKAGE}_SOURCE_PREFIX}" STREQUAL "")
      if("${${PACKAGE}_DIR}" STREQUAL "")
        message(
          FATAL_ERROR
            "Your ${PACKAGE} does not define where to find the data, please update."
        )
      else()
        set(${PACKAGE_PATH_VAR} "${${PACKAGE}_DIR}/..")
      endif()
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

mc_rtc_set_prefix(PLUGINS mc_plugins)

macro(add_plugin plugin)
  cmake_parse_arguments(ADD_PLUGIN "AUTOLOAD" "" "" ${ARGN})
  option(AUTOLOAD_${plugin}_PLUGIN "Automatically load ${plugin} plugin"
         ${ADD_PLUGIN_AUTOLOAD}
  )
  add_library(${plugin} SHARED ${ADD_PLUGIN_UNPARSED_ARGUMENTS})
  set_target_properties(${plugin} PROPERTIES PREFIX "")
  target_link_libraries(${plugin} PUBLIC mc_rtc::mc_control)
  install(
    TARGETS ${plugin}
    ARCHIVE
      DESTINATION
        "${MC_PLUGINS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    LIBRARY
      DESTINATION
        "${MC_PLUGINS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    RUNTIME
      DESTINATION
        "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
  )
  set(plugin_CFG "${CMAKE_CURRENT_SOURCE_DIR}/etc/${plugin}.yaml")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/stage/autoload/${plugin}.yaml" "${plugin}")
  set(AUTOLOAD_DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/autoload/")
  if(AUTOLOAD_${plugin}_PLUGIN)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/stage/autoload/${plugin}.yaml"
            DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/autoload/"
    )
  else()
    set(AUTOLOAD_DESTINATION "${AUTOLOAD_DESTINATION}/${plugin}.yaml")
    install(
      CODE "
    if(EXISTS \"${AUTOLOAD_DESTINATION}\")
      message(STATUS \"Removing: ${AUTOLOAD_DESTINATION}\")
      file(REMOVE \"${AUTOLOAD_DESTINATION}\")
    endif()"
    )
  endif()
endmacro()

file(GLOB plugin_targets "${CMAKE_CURRENT_LIST_DIR}/plugins/*Targets.cmake")
foreach(plugin ${plugin_targets})
  include(${plugin})
endforeach()
