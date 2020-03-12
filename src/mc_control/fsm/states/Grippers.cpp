/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Grippers.h>
#include <mc_filter/utils/clamp.h>

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
  if(config_.has("grippers"))
  {
    auto grippers = config_("grippers");
    for(const auto & g : grippers.keys())
    {
      if(ctl.grippers.count(g) == 0)
      {
        LOG_WARNING("[FSM::" << name() << "] " << g << " is not a known gripper")
        continue;
      }
      auto & gripper = ctl.grippers[g];
      grippers(g)("percentVMAX", gripper->percentVMAX());
      if(grippers(g).has("safety"))
      {
        const auto & safety = grippers(g)("safety");
        if(safety.has("threshold"))
        {
          gripper->actualCommandDiffTrigger(static_cast<double>(safety("threshold")) * M_PI / 180);
        }
        if(grippers(g).has("iter"))
        {
          gripper->overCommandLimitIterN(safety("iter"));
        }
        if(grippers(g).has("release"))
        {
          gripper->releaseSafetyOffset(static_cast<double>(safety("release")) * M_PI / 180);
        }
      }

      if(grippers(g).has("opening"))
      {
        double open = mc_filter::utils::clamp(static_cast<double>(grippers(g)("opening")), 0, 1);
        gripper->setTargetOpening(open);
        grippers_.push_back(g);
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
        grippers_.push_back(g);
      }
      else
      {
        LOG_WARNING("[FSM::" << name() << "] " << g << " has no opening or target specified")
      }
    }
  }
}

bool Grippers::run(Controller & ctl)
{
  if(std::all_of(grippers_.begin(), grippers_.end(),
                 [&ctl](const std::string & g) { return ctl.grippers[g]->complete(); }))
  {
    output("OK");
    return true;
  }
  return false;
}

void Grippers::teardown(Controller & ctl)
{
  if(config_.has("grippers"))
  {
    auto grippers = config_("grippers");
    for(const auto & g : grippers.keys())
    {
      ctl.grippers[g]->resetDefaults();
    }
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Grippers", mc_control::fsm::Grippers)
