#include "mc_halfsitpose_controller.h"

#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

/* Common stuff */
MCHalfSitPoseController::MCHalfSitPoseController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt), halfSitPose(robot().mbc().q)
{

  /* Set the halfSitPose in posture Task */
  const auto & halfSit = robot_module->stance();
  const auto & ref_joint_order = robot().refJointOrder();
  for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
  {
    if(robot().hasJoint(ref_joint_order[i]))
    {
      halfSitPose[robot().jointIndexByName(ref_joint_order[i])] = halfSit.at(ref_joint_order[i]);
    }
  }

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

void MCHalfSitPoseController::reset(const ControllerResetData & reset_data)
{
  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  postureTask->reset();
  postureTask.get()->weight(100.);
  postureTask.get()->stiffness(2.);
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
  gui_->addElement({"Controller"},
                   mc_rtc::gui::Button("Go half-sitting", [this]() { postureTask->posture(halfSitPose); }));
}

} // namespace mc_control
