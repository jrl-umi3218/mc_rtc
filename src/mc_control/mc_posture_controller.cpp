#include <mc_control/mc_posture_controller.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

/* Common stuff */
MCPostureController::MCPostureController()
: MCController()
{
  qpsolver->setContacts({});
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->solver.addTask(postureTask.get());

  std::cout << "MCPostureController init done " << this << std::endl;
}

}
