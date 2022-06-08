/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_external_forces_controller.h"

MULTI_CONTROLLERS_CONSTRUCTOR("ExternalForces",
                              ExternalForcesController(rm, dt, config, mc_control::MCController::Backend::Tasks),
                              "ExternalForces_TVM",
                              ExternalForcesController(rm, dt, config, mc_control::MCController::Backend::TVM))
