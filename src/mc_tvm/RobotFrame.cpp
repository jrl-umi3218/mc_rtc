/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/RobotFrame.h>

#include <mc_tvm/Robot.h>

#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

RobotFrame::RobotFrame(NewRobotFrameToken tkn, const mc_rbdyn::RobotFrame & frame)
: Frame(tkn, frame), jac_(frame.robot().mb(), frame.body()), blocks_(jac_.compactPath(frame.robot().mb())),
  jacTmp_(6, jac_.dof()), jacobian_(6, frame.robot().mb().nrDof()), jacDot_(jacobian_)
{
  // clang-format off
  registerUpdates(
                  Update::Jacobian, &RobotFrame::updateJacobian,
                  Update::NormalAcceleration, &RobotFrame::updateNormalAcceleration,
                  Update::JDot, &RobotFrame::updateJDot);
  // clang-format off

  auto & robot_ = frame.robot().tvmRobot();

  addOutputDependency<RobotFrame>(Output::Jacobian, Update::Jacobian);
  addInputDependency<RobotFrame>(Update::Jacobian, robot_, Robot::Output::FV);

  addOutputDependency<RobotFrame>(Output::NormalAcceleration, Update::NormalAcceleration);
  addInputDependency<RobotFrame>(Update::NormalAcceleration, robot_, Robot::Output::NormalAcceleration);

  addOutputDependency<RobotFrame>(Output::JDot, Update::JDot);
  addInputDependency<RobotFrame>(Update::JDot, robot_, Robot::Output::FV);

  addInternalDependency<RobotFrame>(Update::NormalAcceleration, Update::Jacobian); // for h_
  addInternalDependency<RobotFrame>(Update::NormalAcceleration, Update::Velocity);
  addInternalDependency<RobotFrame>(Update::JDot, Update::Velocity);
  // Not strictly true but they use the same internal variable, so in case the
  // graph gets parallelized and we start to use JDot...
  addInternalDependency<RobotFrame>(Update::JDot, Update::Jacobian);
  addInternalDependency<RobotFrame>(Update::JDot, Update::NormalAcceleration);
}

void RobotFrame::updateJacobian()
{
  const auto & robot = frame().robot();
  h_ = -mc_rbdyn::hat(robot.mbc().bodyPosW[frame().bodyMbcIndex()].rotation().transpose() * frame().X_b_f().translation());
  const auto & partialJac = jac_.jacobian(robot.mb(), robot.mbc());
  jacTmp_ = partialJac;
  jacTmp_.bottomRows<3>().noalias() += h_ * partialJac.topRows<3>();
  jacobian_.setZero();
  jac_.addFullJacobian(blocks_, jacTmp_, jacobian_);
}

void RobotFrame::updateNormalAcceleration()
{
  const auto & robot = frame().robot();
  const auto & tvm_robot = robot.tvmRobot();
  normalAcceleration_ = jac_.normalAcceleration(robot.mb(), robot.mbc(), tvm_robot.normalAccB());
  normalAcceleration_.linear().noalias() += h_ * normalAcceleration_.angular() + velocity_.angular().cross(h_ * velocity_.angular());
}

void RobotFrame::updateJDot()
{
  const auto & robot = frame().robot();
  const auto & partialJac = jac_.jacobianDot(robot.mb(), robot.mbc());
  jacTmp_ = partialJac;
  jacTmp_.bottomRows<3>().noalias() += h_ * partialJac.topRows<3>();
  jacTmp_.bottomRows<3>().noalias() -= mc_rbdyn::hat(h_*velocity_.angular()) * jac_.jacobian(robot.mb(), robot.mbc()).topRows<3>();
  jacDot_.setZero();
  jac_.addFullJacobian(blocks_, jacTmp_, jacDot_);
}

} // namespace mc_tvm
