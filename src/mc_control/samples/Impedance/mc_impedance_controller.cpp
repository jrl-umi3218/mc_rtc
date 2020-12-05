/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_impedance_controller.h"

#include <mc_rtc/constants.h>

namespace mc_control
{

MCImpedanceController::MCImpedanceController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(*compoundJointConstraint);

  // add PostureTask
  qpsolver->addTask(postureTask);
  postureTask->stiffness(1);
  postureTask->weight(1);

  // add EndEffectorTask for root link
  rootLinkTask =
      std::make_shared<mc_tasks::EndEffectorTask>("base_link", robots(), robots().robotIndex(), 100.0, 10000.0);
  rootLinkTask->set_ef_pose(sva::PTransformd(Eigen::Vector3d(0, 0, 1)));
  solver().addTask(rootLinkTask);

  // add ImpedanceTask of left hand
  Eigen::Vector3d posM = Eigen::Vector3d::Constant(1.0);
  Eigen::Vector3d posK = Eigen::Vector3d(100.0, 100.0, 1000.0);
  Eigen::Vector3d posD = 2 * posM.cwiseProduct(posK).cwiseSqrt();
  impedanceTask =
      std::make_shared<mc_tasks::force::ImpedanceTask>("LeftGripper", robots(), robots().robotIndex(), 100.0);
  impedanceTask->impedancePosition(posM, posD, posK);
  impedanceTask->impedanceOrientation(100 * posM, 100 * posD, 100 * posK);
  impedanceTask->wrenchGain(sva::MotionVecd(Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones()));
}

void MCImpedanceController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  impedanceTask->reset();
  solver().addTask(impedanceTask);
}

bool MCImpedanceController::run()
{
  angle += 3.0 * solver().dt();

  // Track the circle trajectory
  impedanceTask->desiredPose({sva::RotY(mc_rtc::constants::PI),
                              center + Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), 0)});

  return mc_control::MCController::run();
}

} // namespace mc_control
