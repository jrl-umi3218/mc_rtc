/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Grippers.h>
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/constants.h>

namespace mc_control
{

namespace fsm
{

void Grippers::start(Controller & ctl)
{
  unsigned int rIndex = 0;
  config_("keepSafetyConfig", keepSafetyConfig_);
  if(config_.has("robot"))
  {
    if(!ctl.robots().hasRobot(config_("robot")))
    {
      std::string robot = config_("robot");
      mc_rtc::log::warning("[FSM::{}] Configured for robot {} but this robot is not part of the controller", name(),
                           robot);
      return;
    }
    rIndex = ctl.robots().robotIndex(config_("robot"));
  }
  if(config_.has("grippers"))
  {
    auto grippers = config_("grippers");
    auto & ctl_grippers = ctl.robots().robot(rIndex).grippersByName();
    for(const auto & g : grippers.keys())
    {
      if(ctl_grippers.count(g) == 0)
      {
        mc_rtc::log::warning("[FSM::{}] {} is not a known gripper", name(), g);
        continue;
      }
      auto & gripper = ctl_grippers.at(g);
      if(!keepSafetyConfig_)
      {
        gripper->saveConfig();
      }
      gripper->configure(grippers(g));
      grippers_.push_back(std::ref(*gripper));
    }
  }
}

bool Grippers::run(Controller &)
{
  if(std::all_of(grippers_.begin(), grippers_.end(),
                 [](const mc_control::GripperRef & g) { return g.get().complete(); }))
  {
    output("OK");
    return true;
  }
  return false;
}

void Grippers::teardown(Controller &)
{
  for(auto & g : grippers_)
  {
    if(keepSafetyConfig_)
    { // Make the current safety configuration the new default
      g.get().saveConfig();
    }
    else
    {
      g.get().restoreConfig();
    }
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Grippers", mc_control::fsm::Grippers)
