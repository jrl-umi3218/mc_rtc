#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
#

macro(add_plugin plugin)
  set(plugin_SRC ${plugin}/${plugin}.cpp)
  set(plugin_HDR ${plugin}/${plugin}.h)
  add_library(${plugin} SHARED ${plugin_SRC} ${plugin_HDR})
  set_target_properties(${plugin} PROPERTIES FOLDER plugins)
  set_target_properties(${plugin} PROPERTIES PREFIX "")
  target_link_libraries(${plugin} mc_control)
  install(TARGETS ${plugin} DESTINATION ${MC_PLUGINS_INSTALL_PREFIX})
  set(plugin_CFG "${CMAKE_CURRENT_SOURCE_DIR}/${plugin}/etc/${plugin}.yaml")
  if(EXISTS ${plugin_CFG})
    install(FILES ${plugin_CFG} DESTINATION ${MC_PLUGINS_INSTALL_PREFIX}/etc)
  else()
  endif()
endmacro()

if(NOT ${DISABLE_ROS} AND ${ROSCPP_FOUND})
  add_plugin(ROS)
endif()
