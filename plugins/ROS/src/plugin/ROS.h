/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

#include "Services.h"

namespace mc_plugin
{

struct ROSPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  inline void before(mc_control::MCGlobalController &) override {}

  void after(mc_control::MCGlobalController & controller) override;

  ~ROSPlugin() override;

private:
  bool publish_control = true;
  bool publish_env = true;
  bool publish_real = true;
  double publish_timestep = 0.01;
  size_t published_env = 0;
  std::unique_ptr<ROSServices> services_;

  void build(mc_control::MCGlobalController & controller);
};

} // namespace mc_plugin
