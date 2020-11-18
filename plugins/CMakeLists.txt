#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
#

macro(add_plugin plugin plugin_SRC plugin_HDR)
  option(AUTOLOAD_${plugin}_PLUGIN "Automatically load ${plugin} plugin" ON)
  if(MC_RTC_BUILD_STATIC)
    target_sources(mc_control PRIVATE ${plugin_SRC} ${plugin_HDR})
  else()
    add_library(${plugin} SHARED ${plugin_SRC} ${plugin_HDR})
    set_target_properties(${plugin} PROPERTIES PREFIX "")
    target_link_libraries(${plugin} PUBLIC mc_rtc::mc_control)
    install(TARGETS ${plugin}
      ARCHIVE DESTINATION "${MC_PLUGINS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
      LIBRARY DESTINATION "${MC_PLUGINS_LIBRARY_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
      RUNTIME DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}$<$<CONFIG:debug>:/debug>"
    )
  endif()
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/autoload/${plugin}.yaml" "${plugin}")
  if(AUTOLOAD_${plugin}_PLUGIN)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/autoload/${plugin}.yaml"
      DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/autoload/")
  endif()
endmacro()

file(GLOB plugin_dirs "*")
foreach(plugin_dir ${plugin_dirs})
  if(EXISTS ${plugin_dir}/CMakeLists.txt)
    add_subdirectory(${plugin_dir})
  endif()
endforeach()
