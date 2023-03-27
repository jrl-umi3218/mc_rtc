#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

macro(add_plugin plugin)
  option(AUTOLOAD_${plugin}_PLUGIN "Automatically load ${plugin} plugin" ON)
  if(MC_RTC_BUILD_STATIC)
    target_sources(mc_control PRIVATE ${ARGN})
  else()
    add_library(${plugin} SHARED ${ARGN})
    set_target_properties(${plugin} PROPERTIES PREFIX "")
    target_link_libraries(${plugin} PUBLIC mc_rtc::mc_control)
    install(TARGETS ${plugin}
      ARCHIVE DESTINATION "${MC_PLUGINS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
      LIBRARY DESTINATION "${MC_PLUGINS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
      RUNTIME DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:${MC_RTC_LOADER_DEBUG_SUFFIX}>"
    )
  endif()
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/stage/autoload/${plugin}.yaml" "${plugin}")
  if(AUTOLOAD_${plugin}_PLUGIN)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/stage/autoload/${plugin}.yaml"
      DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/autoload/")
  endif()
endmacro()

file(GLOB plugin_dirs "*")
foreach(plugin_dir ${plugin_dirs})
  if(EXISTS ${plugin_dir}/CMakeLists.txt)
    add_subdirectory(${plugin_dir})
  endif()
endforeach()
