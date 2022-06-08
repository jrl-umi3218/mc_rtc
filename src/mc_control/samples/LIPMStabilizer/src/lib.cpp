/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_lipm_stabilizer.h"

MULTI_CONTROLLERS_CONSTRUCTOR("LIPMStabilizer",
                              LIPMStabilizerController(rm, dt, config, mc_control::MCController::Backend::Tasks),
                              "LIPMStabilizer_TVM",
                              LIPMStabilizerController(rm, dt, config, mc_control::MCController::Backend::TVM))
