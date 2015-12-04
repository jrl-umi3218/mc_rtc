#ifndef _H_CONTACTSENSOR_H_
#define _H_CONTACTSENSOR_H_

#include <mc_control/mc_controller.h>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI ContactSensor
{
  virtual void resetOffset() {}

  virtual std::vector<std::string> update(MCController & ctl) = 0;
};

}

#endif
