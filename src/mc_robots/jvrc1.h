/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/RobotModule.h>

#include "api.h"

namespace mc_robots
{

struct JVRC1DummySpeaker : public mc_rbdyn::Device
{
  inline JVRC1DummySpeaker() : JVRC1DummySpeaker("") {}
  inline JVRC1DummySpeaker(const std::string & name) : mc_rbdyn::Device(name) { type_ = "JVRC1DummySpeaker"; }
  ~JVRC1DummySpeaker() override = default;

  /** Return the text to say and reset the initial state */
  inline const std::string say()
  {
    std::string out = "";
    std::swap(out, text_);
    return out;
  }

  /** Set the text to be said */
  inline void say(const std::string & text) { text_ = text; }

  /** Return true if text to say is not empty */
  inline bool hasSomethingToSay() { return text_ != ""; }

  mc_rbdyn::DevicePtr clone() const override;

private:
  std::string text_ = "";
};

struct MC_ROBOTS_DLLAPI JVRC1RobotModule : public mc_rbdyn::RobotModule
{
public:
  JVRC1RobotModule(bool fixed = false, bool filter_mimics = false);
};

} // namespace mc_robots
