/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/Grippers.h>
#include <mc_filter/utils/clamp.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/io_utils.h>

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

      if(grippers(g).has("opening"))
      {
        try
        {
          std::map<std::string, double> jointsOpening = grippers(g)("opening");
          for(const auto & jOpen : jointsOpening)
          {
            const auto & jName = jOpen.first;
            if(gripper->hasActiveJoint(jName))
            {
              gripper->setTargetOpening(jName, jOpen.second);
            }
            else
            {
              mc_rtc::log::warning("[{}] Attempted to set target opening for joint \"{}\" which is not part of the "
                                   "gripper's \"{}\" active joints ([{}])",
                                   name(), jName, g, mc_rtc::io::to_string(gripper->activeJoints()));
            }
          }
          grippers_.push_back(std::ref(*gripper));
        }
        catch(mc_rtc::Configuration::Exception & e)
        {
          e.silence();
          try
          {
            double open = grippers(g)("opening");
            gripper->setTargetOpening(open);
            grippers_.push_back(std::ref(*gripper));
          }
          catch(mc_rtc::Configuration::Exception & e)
          {
            e.silence();
            mc_rtc::log::warning("[{}] Target gripper opening for gripper \"{}\" must either be a map<Joint name "
                                 "(string), opening (double)> or a double value",
                                 name(), g);
          }
        }
      }
      else if(grippers(g).has("target"))
      {
        try
        {
          std::map<std::string, double> jointTargets = grippers(g)("target");
          for(const auto & jTarget : jointTargets)
          {
            const auto & jName = jTarget.first;
            if(gripper->hasActiveJoint(jName))
            {
              gripper->setTargetQ(jName, jTarget.second);
            }
            else
            {
              mc_rtc::log::warning("[{}] Attempted to set target opening for joint \"{}\" which is not part of the "
                                   "gripper's \"{}\" active joints ([{}])",
                                   name(), jName, g, mc_rtc::io::to_string(gripper->activeJoints()));
            }
          }
          grippers_.push_back(std::ref(*gripper));
        }
        catch(mc_rtc::Configuration::Exception & e)
        {
          e.silence();
          try
          {
            std::vector<double> target = grippers(g)("target");
            if(gripper->activeJoints().size() != target.size())
            {
              mc_rtc::log::warning(
                  "[FSM::{}] Provided target for {} does not have the correct size (expected: {}, got: {})", name(), g,
                  gripper->curPosition().size(), target.size());
              continue;
            }
            gripper->setTargetQ(target);
            grippers_.push_back(std::ref(*gripper));
          }
          catch(mc_rtc::Configuration::Exception & e)
          {
            e.silence();
            mc_rtc::log::warning("[{}] Target for gripper \"{}\" must either be a map<joint name (string), target "
                                 "(double)> or a vector<double> of size {}",
                                 name(), g, gripper->activeJoints().size());
          }
        }
      }
      else
      {
        mc_rtc::log::warning("[FSM::{}] {} has no opening or target specified", name(), g);
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
