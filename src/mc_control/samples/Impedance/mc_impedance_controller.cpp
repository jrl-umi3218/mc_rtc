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
  if(robot().mb().joint(0).type() != rbd::Joint::Type::Fixed)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "The \"Impedance\" controller requires a robot with a fixed base (base of robot {} has {} DoF)", robot().name(),
        robot().mb().joint(0).dof());
  }

  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(*compoundJointConstraint);

  // add PostureTask
  qpsolver->addTask(postureTask);
  postureTask->stiffness(1);
  postureTask->weight(1);

  // add ImpedanceTask of left hand
  Eigen::Vector3d posM = Eigen::Vector3d::Constant(1.0);
  Eigen::Vector3d posK = Eigen::Vector3d(100.0, 100.0, 1000.0);
  Eigen::Vector3d posD = 2 * posM.cwiseProduct(posK).cwiseSqrt();
  impedanceTask_ =
      std::make_shared<mc_tasks::force::ImpedanceTask>("LeftGripper", robots(), robots().robotIndex(), 100.0);
  impedanceTask_->impedancePosition(posM, posD, posK);
  impedanceTask_->impedanceOrientation(100 * posM, 100 * posD, 100 * posK);
  impedanceTask_->wrenchGain(sva::MotionVecd(Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones()));
}

void MCImpedanceController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  impedanceTask_->reset();
  solver().addTask(impedanceTask_);
  center_ = impedanceTask_->desiredPose().translation() + Eigen::Vector3d{0.0, radius_, 0.0};
  orientation_ = impedanceTask_->desiredPose().rotation();
  angle_ = 3 * mc_rtc::constants::PI / 2.;
}

bool MCImpedanceController::run()
{

  // Track the circle trajectory
  impedanceTask_->desiredPose(
      {orientation_, center_ + Eigen::Vector3d(radius_ * std::cos(angle_), radius_ * std::sin(angle_), 0)});
  angle_ += speed_ * solver().dt();

  return mc_control::MCController::run();
}

} // namespace mc_control
