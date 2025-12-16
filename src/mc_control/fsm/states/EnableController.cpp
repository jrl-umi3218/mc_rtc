/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/EnableController.h>

namespace mc_control
{

namespace fsm
{

void EnableControllerState::start(Controller & ctl)
{
  std::string next_controller = config_("NextController", std::string(""));
  if(ctl.datastore().call<bool, const std::string &>("Global::EnableController", next_controller)) { output("OK"); }
  else
  {
    output("Failed");
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("EnableController", mc_control::fsm::EnableControllerState)
