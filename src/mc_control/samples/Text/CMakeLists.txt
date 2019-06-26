#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

add_controller("text"
  "mc_text_controller.cpp"
  "mc_text_controller.h"
)

install(FILES etc/Text.conf
        DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/etc/)
