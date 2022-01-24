/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>

struct MC_CONTROL_FSM_STATE_DLLAPI ConfigureState : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller &) override {}

  bool run(mc_control::fsm::Controller &) override
  {
    return false;
  }

  void teardown(mc_control::fsm::Controller &) override {}

  unsigned int value() const
  {
    return value_;
  }

  static std::string ExpectedStateName;

protected:
  unsigned int value_ = 0;
};
