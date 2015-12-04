#ifndef _H_MCPOSTURECONTROLLER_H_
#define _H_MCPOSTURECONTROLLER_H_

#include <mc_control/mc_controller.h>

#include <mc_control/api.h>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCPostureController : public MCController
{
public:
  /* Common stuff */
  MCPostureController(double dt);
};

}

#endif
