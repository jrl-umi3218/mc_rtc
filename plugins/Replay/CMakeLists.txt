#
# Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(plugin_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/Replay.cpp")
set(plugin_HDR "${CMAKE_CURRENT_SOURCE_DIR}/src/Replay.h")

set(AUTOLOAD_Replay_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load Replay plugin"
)
add_plugin(Replay ${plugin_SRC} ${plugin_HDR})
