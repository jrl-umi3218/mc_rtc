# -- Path to mc_rtc install prefix
set(PACKAGE_PREFIX_DIR "@MC_RTC_INSTALL_PREFIX@")
message(VERBOSE "mc_rtc is installed in: ${PACKAGE_PREFIX_DIR}")

# -- Library source directory --
set(MC_RTC_SRCDIR "@MC_RTC_SOURCE_DIR@")

# Allow to override mc_rtc debug suffix
if(NOT DEFINED MC_RTC_LOADER_DEBUG_SUFFIX)
  set(MC_RTC_LOADER_DEBUG_SUFFIX "@MC_RTC_LOADER_DEBUG_SUFFIX@")
endif()

set(MC_RTC_BUILD_IN_NIX @MC_RTC_BUILD_IN_NIX@)

# The point of MC_RTC_HONOR_INSTALL_PREFIX is to let the user decide whether they want
# to install the plugins in the same prefix as mc_rtc, or in a dedicated one. This is
# useful for Nix, where we want to let runtime dependencies install themselves into
# their own dedicated prefix, instead of being forced to install into the same prefix as
# mc_rtc.
if(NOT DEFINED MC_RTC_HONOR_INSTALL_PREFIX)
  if(MC_RTC_BUILD_IN_NIX)
    message(
      VERBOSE
      "Defaulting MC_RTC_HONOR_INSTALL_PREFIX=ON because it isn't set and we detected that we're building in Nix (MC_RTC_BUILD_IN_NIX=ON)"
    )
    set(MC_RTC_HONOR_INSTALL_PREFIX ON)
  else()
    set(MC_RTC_HONOR_INSTALL_PREFIX OFF)
  endif()
endif()

if(MC_RTC_HONOR_INSTALL_PREFIX)
  message(
    VERBOSE
    "Honoring CMAKE_INSTALL_PREFIX for mc_rtc installation because MC_RTC_HONOR_INSTALL_PREFIX=ON. This means that downstream packages using the macros such as add_controller, add_robot (...) defined in ${CMAKE_CURRENT_LIST_DIR}/mc_rtcMacros.cmake will be installed in their own CMAKE_INSTALL_PREFIX as opposed to mc_rtc's ${PACKAGE_PREFIX_DIR}"
  )
else()
  message(
    VERBOSE
    "Runtime dependencies installed by macros such as add_controller, add_robot (...) [defined in ${CMAKE_CURRENT_LIST_DIR}/mc_rtcMacros.cmake] will be installed in mc_rtc's install prefix (${PACKAGE_PREFIX_DIR}) because MC_RTC_HONOR_INSTALL_PREFIX=OFF."
  )
endif()

# -- Library install directory --
macro(mc_rtc_set_all_install_paths HONOR_PREFIX)
  if(HONOR_PREFIX)
    message(DEBUG "Honoring CMAKE_INSTALL_PREFIX for all runtime install paths")
    include(GNUInstallDirs)
    set(MC_RTC_BINDIR "${CMAKE_INSTALL_FULL_BINDIR}")
    set(MC_RTC_DOCDIR "${CMAKE_INSTALL_FULL_DOCDIR}")
    set(MC_RTC_LIBDIR "${CMAKE_INSTALL_FULL_LIBDIR}")
  else()
    message(DEBUG "Using mc_rtc's install prefix for all runtime install paths")
    # On Nix all paths obtained from GNUInstallDir are absolute, here we want mc_rtc'
    # runtime paths include(GNUInstallDirs)
    set(MC_RTC_BINDIR "${PACKAGE_PREFIX_DIR}/bin")
    set(MC_RTC_DOCDIR "${PACKAGE_PREFIX_DIR}/share/doc/mc_rtc")
    set(MC_RTC_LIBDIR "${PACKAGE_PREFIX_DIR}/lib")
  endif()
  message(DEBUG
          "MC_RTC_BINDIR set to ${MC_RTC_BINDIR} because HONOR_PREFIX=${HONOR_PREFIX}"
  )
  message(DEBUG
          "MC_RTC_DOCDIR set to ${MC_RTC_DOCDIR} because HONOR_PREFIX=${HONOR_PREFIX}"
  )
  message(DEBUG
          "MC_RTC_LIBDIR set to ${MC_RTC_LIBDIR} because HONOR_PREFIX=${HONOR_PREFIX}"
  )
endmacro()

# -- Helper to set the components install prefix --
macro(mc_rtc_set_prefix NAME FOLDER)
  if(${ARGC} GREATER 2)
    set(HONOR_PREFIX "${ARGV2}")
  else()
    set(HONOR_PREFIX "${MC_RTC_HONOR_INSTALL_PREFIX}")
  endif()
  # Modify the base install path for runtime dependencies: respect
  # MC_RTC_HONOR_INSTALL_PREFIX unless overriden by HONOR_PREFIX as a 3rd argument here.
  # This is necessary as we need to be able to get mc_rtc's install prefix for default
  # states. TODO: we should be saving default state path in mc_rtc build (config.in.h)
  # and loading it by default with a mechanism to clear it instead.
  mc_rtc_set_all_install_paths(${HONOR_PREFIX})
  set(MC_${NAME}_LIBRARY_INSTALL_PREFIX "${MC_RTC_LIBDIR}/${FOLDER}")
  if(WIN32)
    set(MC_${NAME}_RUNTIME_INSTALL_PREFIX "${MC_RTC_BINDIR}/${FOLDER}")
  else()
    set(MC_${NAME}_RUNTIME_INSTALL_PREFIX "${MC_${NAME}_LIBRARY_INSTALL_PREFIX}")
  endif()
  # For backward compatibility
  set(MC_${NAME}_INSTALL_PREFIX "${MC_${NAME}_LIBRARY_INSTALL_PREFIX}")
  message(DEBUG
          "MC_${NAME}_LIBRARY_INSTALL_PREFIX=${MC_${NAME}_LIBRARY_INSTALL_PREFIX}"
  )
  message(DEBUG
          "MC_${NAME}_RUNTIME_INSTALL_PREFIX=${MC_${NAME}_RUNTIME_INSTALL_PREFIX}"
  )
  if(NOT "${MC_RTC_HONOR_INSTALL_PREFIX}" STREQUAL "${HONOR_PREFIX}")
    # restore base install path to the user-specified MC_RTC_HONOR_INSTALL_PREFIX
    mc_rtc_set_all_install_paths(${MC_RTC_HONOR_INSTALL_PREFIX})
  endif()
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

# -- States -- Default MC_RTC_HONOR_INSTALL_PREFIX to OFF, so that mc_rtc's path to its
# default states is available to controllers
mc_rtc_set_prefix(STATES_DEFAULT mc_controller/fsm/states OFF)
# Honour MC_RTC_HONOR_INSTALL_PREFIX for the main states prefix, so that users can
# choose to install their own states in a different prefix if they want to
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

macro(find_description_package)
  set(options OPTIONAL)
  cmake_parse_arguments(arg_package "${options}" "" "" ${ARGN})

  list(GET arg_package_UNPARSED_ARGUMENTS 0 _package_name)
  if(NOT _package_name)
    message(FATAL_ERROR "PACKAGE must be specified.")
  endif()

  set(PACKAGE_OPTION REQUIRED)
  if(arg_package_OPTIONAL)
    set(PACKAGE_OPTION OPTIONAL)
  endif()

  set(PACKAGE_PATH_VAR "${_package_name}_PATH")
  string(TOUPPER "${PACKAGE_PATH_VAR}" PACKAGE_PATH_VAR)
  find_package(${_package_name} ${PACKAGE_OPTION})
  if("${${_package_name}_INSTALL_PREFIX}" STREQUAL "")
    if("${${_package_name}_SOURCE_PREFIX}" STREQUAL "")
      if("${${_package_name}_DIR}" STREQUAL "")
        message(
          FATAL_ERROR
            "Your ${_package_name} does not define where to find the data, please update."
        )
      else()
        set(${PACKAGE_PATH_VAR} "${${_package_name}_DIR}/..")
      endif()
    else()
      set(${PACKAGE_PATH_VAR} "${${_package_name}_SOURCE_PREFIX}")
    endif()
  else()
    set(${PACKAGE_PATH_VAR} "${${_package_name}_INSTALL_PREFIX}")
  endif()
  # Cleanup the path provided by CMake
  get_filename_component(${PACKAGE_PATH_VAR} "${${PACKAGE_PATH_VAR}}" REALPATH)
  message("-- Found ${_package_name}: ${${PACKAGE_PATH_VAR}}")
endmacro()

# -- Helper to create a new mc_rtc plugin

mc_rtc_set_prefix(PLUGINS mc_plugins)

macro(add_plugin plugin)
  cmake_parse_arguments(ADD_PLUGIN "AUTOLOAD" "" "" ${ARGN})
  jrl_option(
    AUTOLOAD_${plugin}_PLUGIN "Automatically load ${plugin} plugin"
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
