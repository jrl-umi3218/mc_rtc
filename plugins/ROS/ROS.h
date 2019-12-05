/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

namespace mc_plugins
{

struct ROSPlugin : public mc_control::GlobalPlugin
{
  void init(const mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(const mc_control::MCGlobalController & controller) override;

  inline void before(const mc_control::MCGlobalController &) override {}

  void after(const mc_control::MCGlobalController & controller) override;

private:
  bool publish_control = true;
  bool publish_env = true;
  bool publish_real = true;
  double publish_timestep = 0.01;
};

} // namespace mc_plugins
