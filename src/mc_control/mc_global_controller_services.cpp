/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/ros.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

bool MCGlobalController::GoToHalfSitPose_service()
{
  if(controller_)
  {
    return GoToHalfSitPose();
  }
  else
  {
    return false;
  }
}

} // namespace mc_control
