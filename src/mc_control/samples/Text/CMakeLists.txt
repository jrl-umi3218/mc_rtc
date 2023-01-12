#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

add_controller("text"
  "${CMAKE_CURRENT_SOURCE_DIR}/mc_text_controller.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/mc_text_controller.h"
)

configure_file(etc/Text.conf "${CMAKE_CURRENT_BINARY_DIR}/etc/Text_TVM.conf")

install(FILES etc/Text.conf "${CMAKE_CURRENT_BINARY_DIR}/etc/Text_TVM.conf"
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc/)
