#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

get_cython_libraries(mc_rtc cython_libs)
get_python_names(python_names)

list(LENGTH python_names NSTATES)
math(EXPR NSTATES "${NSTATES} - 1")
foreach(i RANGE ${NSTATES})
  list(GET python_names ${i} PYTHON_NAME)
  list(GET cython_libs ${i} CYTHON_LIB)
  string(TOLOWER ${PYTHON_NAME} PYTHON_NAME_LOWER)
  set(SRC "${CMAKE_CURRENT_BINARY_DIR}/generated/${PYTHON_NAME_LOWER}_controller.cpp")
  configure_file(python_controller.cpp "${SRC}")
  add_controller(${PYTHON_NAME_LOWER}_controller "${SRC}" "python_controller.h")
  set_target_properties(${PYTHON_NAME_LOWER}_controller PROPERTIES OUTPUT_NAME ${PYTHON_NAME_LOWER})
  target_link_libraries(${PYTHON_NAME_LOWER}_controller PRIVATE ${CYTHON_LIB})
  target_include_directories(${PYTHON_NAME_LOWER}_controller PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
endforeach()
