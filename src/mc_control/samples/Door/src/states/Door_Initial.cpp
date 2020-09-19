#include <mc_control/fsm/Controller.h>

#include "Door_Initial.h"

void Door_Initial::configure(const mc_rtc::Configuration &) {}

void Door_Initial::start(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->addElement({}, mc_rtc::gui::Button("Open door", [this]() { openDoor_ = true; }));
}

bool Door_Initial::run(mc_control::fsm::Controller &)
{
  if(openDoor_)
  {
    output("OpenDoor");
    return true;
  }
  return false;
}

void Door_Initial::teardown(mc_control::fsm::Controller &) {}

EXPORT_SINGLE_STATE("Door_Initial", Door_Initial)
