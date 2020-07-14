/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_observers/Observer.h>

namespace mc_observers
{
Observer::Observer(const std::string & name, const mc_rtc::Configuration &) : name_(name), desc_(name) {}

Observer::~Observer() {}

const std::string & Observer::name() const
{
  return name_;
}

const std::string & Observer::desc() const
{
  return desc_;
}

void Observer::removeFromGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category)
{
  category.push_back(name());
  gui.removeCategory(category);
}

} // namespace mc_observers
