#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
#

macro(add_plugin plugin plugin_SRC plugin_HDR)
  add_library(${plugin} SHARED ${plugin_SRC} ${plugin_HDR})
  set_target_properties(${plugin} PROPERTIES PREFIX "")
  target_link_libraries(${plugin} PUBLIC mc_rtc::mc_control)
  install(TARGETS ${plugin} DESTINATION ${MC_PLUGINS_INSTALL_PREFIX})
endmacro()

function(check_as_needed_linker_flag)
  set(CMAKE_REQUIRED_FLAGS "-Wl,--as-needed")
  CHECK_C_COMPILER_FLAG("" HAS_AS_NEEDED_LINKER_FLAG)
  set(HAS_AS_NEEDED_LINKER_FLAG ${HAS_AS_NEEDED_LINKER_FLAG} CACHE INTERNAL "Linker support --as-needed")
endfunction()

check_as_needed_linker_flag()

file(GLOB plugin_dirs "*")
foreach(plugin_dir ${plugin_dirs})
  if(EXISTS ${plugin_dir}/CMakeLists.txt)
    add_subdirectory(${plugin_dir})
  endif()
endforeach()
set(PACKAGE_EXTRA_MACROS "${PACKAGE_EXTRA_MACROS}" PARENT_SCOPE)
