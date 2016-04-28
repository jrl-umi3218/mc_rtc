#include "mc_halfsitpose_controller.h"

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_rbdyn/RobotModule.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

/* Common stuff */
MCHalfSitPoseController::MCHalfSitPoseController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt),
  halfSitPose(robot().mbc().q)
{

  LOG_INFO("Before Collision")
  /* Set the halfSitPose in posture Task */
  const auto & halfSit = robot_module->stance();
  LOG_INFO("Before Compute")
  for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
  {
    halfSitPose[robot().jointIndexByName(ref_joint_order[i])] = halfSit.at(ref_joint_order[i]);
  }
  LOG_INFO("After Compute")

  selfCollisionConstraint.reset();
  qpsolver->addConstraintSet(selfCollisionConstraint);
  /* Get the complete collision constraint set */
  selfCollisionConstraint.addCollisions(solver(), robot_module->commonSelfCollisions());
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addTask(postureTask.get());
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->setContacts({});
  LOG_SUCCESS("MCHalfSitPoseController init done " << this)
}

bool MCHalfSitPoseController::run()
{
  bool ret = MCController::run();
  postureTask->posture(halfSitPose);
  postureTask.get()->weight(100.);
  return ret;
}

}
