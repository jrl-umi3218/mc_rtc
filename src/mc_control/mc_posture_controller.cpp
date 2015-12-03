#include <mc_control/mc_posture_controller.h>

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

/* Common stuff */
MCPostureController::MCPostureController(double dt)
: MCController(dt)
{
  qpsolver->setContacts({});
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->solver.addTask(postureTask.get());

  LOG_SUCCESS("MCPostureController init done " << this)
}

}
