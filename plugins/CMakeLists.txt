#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
#

macro(add_plugin plugin plugin_SRC plugin_HDR)
  option(AUTOLOAD_${plugin}_PLUGIN "Automatically load ${plugin} plugin" ON)
  add_library(${plugin} SHARED ${plugin_SRC} ${plugin_HDR})
  set_target_properties(${plugin} PROPERTIES PREFIX "")
  target_link_libraries(${plugin} PUBLIC mc_rtc::mc_control)
  install(TARGETS ${plugin} DESTINATION ${MC_PLUGINS_INSTALL_PREFIX})
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/autoload/${plugin}.yaml" "${plugin}")
  if(AUTOLOAD_${plugin}_PLUGIN)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/autoload/${plugin}.yaml"
      DESTINATION "${MC_PLUGINS_INSTALL_PREFIX}/autoload/")
  endif()
endmacro()

file(GLOB plugin_dirs "*")
foreach(plugin_dir ${plugin_dirs})
  if(EXISTS ${plugin_dir}/CMakeLists.txt)
    add_subdirectory(${plugin_dir})
  endif()
endforeach()
