#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

macro(add_robot robot_base)
  set(robot_SRC ${robot_base}.cpp)
  set(robot_HDR ../../include/mc_robots/api.h ${robot_base}.h)
  add_library(${robot_base} SHARED ${robot_SRC} ${robot_HDR})
  set_target_properties(${robot_base} PROPERTIES COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS" PREFIX "")
  target_link_libraries(${robot_base} PUBLIC mc_rbdyn)
  target_include_directories(${robot_base} PRIVATE "../../include/mc_robots")
  install(TARGETS ${robot_base} DESTINATION ${MC_ROBOTS_INSTALL_PREFIX})
endmacro()

add_robot(env)
add_robot(int_obj)
add_robot(json)
add_robot(jvrc-1)
if(NOT DEFINED JVRC_DESCRIPTION_PATH)
  set(JVRC_DESCRIPTION_PATH "${MC_ENV_DESCRIPTION_PATH}/../jvrc_description")
endif()
if(NOT EXISTS "${JVRC_DESCRIPTION_PATH}")
  message(SEND_ERROR "JVRC description was not found at: ${JVRC_DESCRIPTION_PATH}, you can set the JVRC_DESCRIPTION_PATH to solve this problem")
endif()
get_filename_component(JVRC_DESCRIPTION_PATH "${JVRC_DESCRIPTION_PATH}" ABSOLUTE)
message("-- jvrc_description: ${JVRC_DESCRIPTION_PATH}")
set_target_properties(jvrc-1 PROPERTIES COMPILE_FLAGS "-DJVRC_DESCRIPTION_PATH=\"${JVRC_DESCRIPTION_PATH}\"")
