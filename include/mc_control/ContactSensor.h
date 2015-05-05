#ifndef _H_CONTACTSENSOR_H_
#define _H_CONTACTSENSOR_H_

#include <mc_control/mc_controller.h>

namespace mc_control
{

struct ContactSensor
{
  virtual void resetOffset() {}

  virtual std::vector<std::string> update(MCController & ctl) = 0;
};

}

#endif
