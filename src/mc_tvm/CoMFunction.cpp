/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/CoMFunction.h>

#include <mc_rbdyn/Robot.h>
#include <mc_tvm/Robot.h>

namespace mc_tvm
{

CoMFunction::CoMFunction(const mc_rbdyn::Robot & robot)
: tvm::function::abstract::Function(3), comAlgo_(robot.tvmRobot().comAlgo())
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &CoMFunction::updateValue,
                  Update::Velocity, &CoMFunction::updateVelocity,
                  Update::Jacobian, &CoMFunction::updateJacobian,
                  Update::NormalAcceleration, &CoMFunction::updateNormalAcceleration,
                  Update::JDot, &CoMFunction::updateJDot);
  // clang-format on
  addOutputDependency<CoMFunction>(Output::Value, Update::Value);
  addOutputDependency<CoMFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<CoMFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<CoMFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  addOutputDependency<CoMFunction>(Output::JDot, Update::JDot);
  auto & tvm_robot = robot.tvmRobot();
  addVariable(tvm_robot.q(), false);
  auto & comAlgo = tvm_robot.comAlgo();
  addInputDependency<CoMFunction>(Update::Value, comAlgo, mc_tvm::CoM::Output::CoM);
  addInputDependency<CoMFunction>(Update::Velocity, comAlgo, mc_tvm::CoM::Output::Velocity);
  addInputDependency<CoMFunction>(Update::Jacobian, comAlgo, mc_tvm::CoM::Output::Jacobian);
  addInputDependency<CoMFunction>(Update::NormalAcceleration, comAlgo, mc_tvm::CoM::Output::NormalAcceleration);
  addInputDependency<CoMFunction>(Update::JDot, comAlgo, mc_tvm::CoM::Output::JDot);
}

void CoMFunction::reset()
{
  com_ = comAlgo_.robot().robot().com();
  refVel_.setZero();
  refAccel_.setZero();
}

void CoMFunction::updateValue()
{
  value_ = comAlgo_.com() - com_;
}

void CoMFunction::updateVelocity()
{
  velocity_ = comAlgo_.velocity() - refVel_;
}

void CoMFunction::updateJacobian()
{
  splitJacobian(comAlgo_.jacobian(), comAlgo_.robot().q());
}

void CoMFunction::updateNormalAcceleration()
{
  normalAcceleration_ = comAlgo_.normalAcceleration() - refAccel_;
}

void CoMFunction::updateJDot()
{
  splitJacobian(comAlgo_.JDot(), comAlgo_.robot().q());
}

} // namespace mc_tvm
