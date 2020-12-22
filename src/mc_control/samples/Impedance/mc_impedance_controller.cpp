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

  addGUI();
}

bool MCImpedanceController::run()
{

  // Track the circle trajectory
  impedanceTask_->desiredPose({orientation_, circleTrajectory(angle_)});
  angle_ += speed_ * solver().dt();

  return mc_control::MCController::run();
}

Eigen::Vector3d MCImpedanceController::circleTrajectory(double angle)
{
  return center_ + Eigen::Vector3d(radius_ * std::cos(angle), radius_ * std::sin(angle), 0);
}

void MCImpedanceController::addGUI()
{
  // Generate samples every 10deg along the circle circumference
  auto circleSamples = std::vector<Eigen::Vector3d>{};
  circleSamples.resize(37);
  for(size_t i = 0; i < circleSamples.size(); i++)
  {
    auto angle = 2 * mc_rtc::constants::PI * static_cast<double>(i) / static_cast<double>(circleSamples.size() - 1);
    circleSamples[i] = circleTrajectory(angle);
  }
  gui()->addElement({"Impedance"}, mc_rtc::gui::Trajectory("Circle Trajectory",
                                                           [circleSamples]() -> const std::vector<Eigen::Vector3d> & {
                                                             return circleSamples;
                                                           }));
}

void MCImpedanceController::stop()
{
  gui()->removeCategory({"Impedance"});
}

} // namespace mc_control
