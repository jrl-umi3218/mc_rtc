#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

macro(add_robot robot_base)
  set(robot_SRC ${robot_base}.cpp)
  set(robot_HDR ../../include/mc_robots/api.h ${robot_base}.h)
  add_library(${robot_base} SHARED ${robot_SRC} ${robot_HDR})
  set_target_properties(${robot_base} PROPERTIES COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS" PREFIX "")
  set_target_properties(${robot_base} PROPERTIES FOLDER robots)
  target_link_libraries(${robot_base} PUBLIC mc_rbdyn)
  target_include_directories(${robot_base} PRIVATE "../../include/mc_robots")
  install(TARGETS ${robot_base} DESTINATION ${MC_ROBOTS_INSTALL_PREFIX})
endmacro()

add_robot(env)
add_robot(json)

macro(generate_aliases PATH VAR NAME FILE_IN FILE_OUT)
  file(GLOB URDF_FILES ${PATH}/urdf/*.urdf)
  set(${VAR})
  foreach(URDF ${URDF_FILES})
    get_filename_component(URDF_NAME "${URDF}" NAME_WE)
    list(APPEND ${VAR} "${NAME}/${URDF_NAME}: [${NAME}, ${PATH}, ${URDF_NAME}]")
  endforeach()
  string(REPLACE ";" "\n" ${VAR} "${${VAR}}")

  set(PATH_OUT "${CMAKE_CURRENT_BINARY_DIR}/aliases/${FILE_OUT}")
  configure_file("${FILE_IN}" "${PATH_OUT}")
  if(CMAKE_CONFIGURATION_TYPES)
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>/aliases/${FILE_OUT}" INPUT "${PATH_OUT}")
  endif()
  install(FILES "${PATH_OUT}" DESTINATION "${MC_ROBOTS_INSTALL_PREFIX}/aliases/")
endmacro()

generate_aliases("${MC_ENV_DESCRIPTION_PATH}" ENV_ALIASES env env_aliases.in.yml env_aliases.yml)
generate_aliases("${MC_INT_OBJ_DESCRIPTION_PATH}" OBJECT_ALIASES object object_aliases.in.yml object_aliases.yml)

find_description_package(jvrc_description)
add_robot(jvrc1)
set_target_properties(jvrc1 PROPERTIES COMPILE_FLAGS "-DMC_ROBOTS_EXPORTS -DJVRC_DESCRIPTION_PATH=\"${JVRC_DESCRIPTION_PATH}\"")
