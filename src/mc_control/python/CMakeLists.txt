#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

foreach(PYTHON_NAME ${PYTHON_BINDING_VERSIONS})
  string(TOLOWER ${PYTHON_NAME} PYTHON_NAME_LOWER)
  set(SRC "${CMAKE_CURRENT_BINARY_DIR}/generated/${PYTHON_NAME_LOWER}_controller.cpp")
  configure_file(python_controller.cpp "${SRC}")
  add_controller(${PYTHON_NAME_LOWER}_controller "${SRC}" "python_controller.h")
  set_target_properties(
    ${PYTHON_NAME_LOWER}_controller PROPERTIES OUTPUT_NAME ${PYTHON_NAME_LOWER}
  )
  add_dependencies(
    ${PYTHON_NAME_LOWER}_controller mc_control_${PYTHON_NAME} mc_rbdyn_${PYTHON_NAME}
  )
  target_include_directories(
    ${PYTHON_NAME_LOWER}_controller
    PRIVATE $<TARGET_PROPERTY:mc_control_${PYTHON_NAME},INTERFACE_INCLUDE_DIRECTORIES>
            $<TARGET_PROPERTY:mc_rbdyn_${PYTHON_NAME},INTERFACE_INCLUDE_DIRECTORIES>
  )
  target_compile_definitions(
    ${PYTHON_NAME_LOWER}_controller
    PRIVATE $<TARGET_PROPERTY:mc_control_${PYTHON_NAME},INTERFACE_COMPILE_DEFINITIONS>
            $<TARGET_PROPERTY:mc_rbdyn_${PYTHON_NAME},INTERFACE_COMPILE_DEFINITIONS>
  )
  target_include_directories(
    ${PYTHON_NAME_LOWER}_controller PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}"
  )
  target_link_libraries(${PYTHON_NAME_LOWER}_controller PRIVATE ${PYTHON_NAME}::Python)
endforeach()
