/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_door_sample_controller.h"

MULTI_CONTROLLERS_CONSTRUCTOR("DoorSample",
                              DoorSampleController(rm, dt, config, mc_control::MCController::Backend::Tasks),
                              "DoorSample_TVM",
                              DoorSampleController(rm, dt, config, mc_control::MCController::Backend::TVM))
