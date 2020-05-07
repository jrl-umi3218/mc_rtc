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

void Grippers::configure(const mc_rtc::Configuration & config)
{
  config_.load(config);
}

void Grippers::start(Controller & ctl)
{
  unsigned int rIndex = 0;
  if(config_.has("robot"))
  {
    if(!ctl.robots().hasRobot(config_("robot")))
    {
      std::string robot = config_("robot");
      LOG_WARNING("[FSM::" << name() << "] Configured for robot " << robot
                           << " but this robot is not part of the controller")
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
        LOG_WARNING("[FSM::" << name() << "] " << g << " is not a known gripper")
        continue;
      }
      auto & gripper = ctl_grippers.at(g);

      if(grippers(g).has("opening"))
      {
        double open = mc_filter::utils::clamp(static_cast<double>(grippers(g)("opening")), 0, 1);
        gripper->setTargetOpening(open);
        grippers_.push_back(std::ref(*gripper));
      }
      else if(grippers(g).has("target"))
      {
        std::vector<double> target = grippers(g)("target");
        if(gripper->curPosition().size() != target.size())
        {
          LOG_WARNING("[FSM::" << name() << "] Provided target for " << g
                               << " does not have the correct size (expected: " << gripper->curPosition().size()
                               << ", got: " << target.size() << ")")
          continue;
        }
        gripper->setTargetQ(target);
        grippers_.push_back(std::ref(*gripper));
      }
      else
      {
        LOG_WARNING("[FSM::" << name() << "] " << g << " has no opening or target specified")
        continue;
      }

      gripper->saveConfig();
      gripper->percentVMAX(grippers(g)("percentVMAX", gripper->percentVMAX()));
      if(grippers(g).has("safety"))
      {
        const auto & safety = grippers(g)("safety");
        if(safety.has("threshold"))
        {
          gripper->actualCommandDiffTrigger(mc_rtc::constants::toRad(safety("threshold")));
        }
        if(safety.has("iter"))
        {
          gripper->overCommandLimitIterN(safety("iter"));
        }
        if(safety.has("release"))
        {
          gripper->releaseSafetyOffset(mc_rtc::constants::toRad(safety("release")));
        }
      }
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
    g.get().restoreConfig();
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Grippers", mc_control::fsm::Grippers)
