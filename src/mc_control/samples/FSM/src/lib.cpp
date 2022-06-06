/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_fsm_controller.h"

MULTI_CONTROLLERS_CONSTRUCTOR("FSM",
                              FSMController(rm, dt, config, mc_control::MCController::Backend::Tasks),
                              "FSM_TVM",
                              FSMController(rm, dt, config, mc_control::MCController::Backend::TVM))
