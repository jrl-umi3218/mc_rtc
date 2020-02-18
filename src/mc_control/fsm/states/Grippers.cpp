/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Grippers.h>

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
      auto gripper = ctl.grippers[g];
      if(grippers(g).has("percentVMAX"))
      {
        gripper->percentVMAX = grippers(g)("percentVMAX");
      }
      else
      {
        gripper->percentVMAX = 0.25;
      }
      if(grippers(g).has("actualCommandDiffTrigger"))
      {
        gripper->actualCommandDiffTrigger = static_cast<double>(grippers(g)("actualCommandDiffTrigger")) * M_PI / 180;
      }
      else
      {
        gripper->actualCommandDiffTrigger = 8 * M_PI / 180;
      }
      if(grippers(g).has("opening"))
      {
        double open = grippers(g)("opening");
        open = open > 1 ? 1 : open < 0 ? 0 : open;
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
  LOG_ERROR("Grippers::State::Run")
  if(std::all_of(grippers_.begin(), grippers_.end(),
                 [&ctl](const std::string & g) { return ctl.grippers[g]->complete(); }))
  {
    LOG_SUCCESS("OK")
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
      auto gripper = ctl.grippers[g];
      gripper->percentVMAX = 0.25;
    }
  }
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("Grippers", mc_control::fsm::Grippers)
