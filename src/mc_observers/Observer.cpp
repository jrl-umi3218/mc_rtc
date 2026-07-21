/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_observers/Observer.h>

namespace mc_observers
{

static thread_local std::string MC_OBSERVER_MODULE_LOADING_LOCATION = "";

void Observer::set_loading_location(std::string_view location)
{
  MC_OBSERVER_MODULE_LOADING_LOCATION = location;
}

Observer::Observer(const std::string & type, double dt)
: loading_location_(MC_OBSERVER_MODULE_LOADING_LOCATION), type_(type), dt_(dt)
{
}

void Observer::removeFromGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

void Observer::removeFromLogger(mc_rtc::Logger & logger, const std::string &)
{
  logger.removeLogEntries(this);
}

} // namespace mc_observers
