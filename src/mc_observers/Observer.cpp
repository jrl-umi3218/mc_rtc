/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_observers/Observer.h>

namespace mc_observers
{
Observer::Observer(const std::string & name, double dt, const mc_rtc::Configuration & config) : name_(name), dt_(dt) {}

Observer::~Observer() {}

const mc_rbdyn::Robots & Observer::robots() const
{
  assert(robots_);
  return *robots_;
}

const mc_rbdyn::Robot & Observer::robot() const
{
  assert(robots_);
  return robots_->robot();
}

const std::string & Observer::name() const
{
  return name_;
}

double Observer::dt() const
{
  return dt_;
}

void Observer::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.removeCategory({"Observers", name()});
}

} // namespace mc_observers
