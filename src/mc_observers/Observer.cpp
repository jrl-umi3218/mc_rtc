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

} // namespace mc_observers
