/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_observers/Observer.h>

namespace mc_observers
{

void Observer::removeFromGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

void Observer::removeFromLogger(mc_rtc::Logger & logger, const std::string &)
{
  logger.removeLogEntries(this);
}

} // namespace mc_observers
