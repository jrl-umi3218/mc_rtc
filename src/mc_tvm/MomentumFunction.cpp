/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/MomentumFunction.h>

#include <mc_tvm/Momentum.h>
#include <mc_tvm/Robot.h>

#include <mc_rbdyn/Robot.h>

namespace mc_tvm
{

MomentumFunction::MomentumFunction(const mc_rbdyn::Robot & robot)
: tvm::function::abstract::Function(6), momentumAlgo_(robot.tvmRobot().momentumAlgo())
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &MomentumFunction::updateValue,
                  Update::Velocity, &MomentumFunction::updateVelocity,
                  Update::Jacobian, &MomentumFunction::updateJacobian,
                  Update::NormalAcceleration, &MomentumFunction::updateNormalAcceleration,
                  Update::JDot, &MomentumFunction::updateJDot);
  // clang-format on
  addOutputDependency<MomentumFunction>(Output::Value, Update::Value);
  addOutputDependency<MomentumFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<MomentumFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<MomentumFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  addOutputDependency<MomentumFunction>(Output::JDot, Update::JDot);
  // FIXME The variable for the momentum should be dot(q), but the way TVM task dynamics are implemented
  // relies on the derivative order of the variables. If dot(q) is given, the dynamics will simply be
  // a proportional.
  addVariable(robot.tvmRobot().q(), false);
  addInputDependency<MomentumFunction>(Update::Value, momentumAlgo_, mc_tvm::Momentum::Output::Momentum);
  // Still add momentum velocity dependency so that it is computed in the graph for logging
  addInputDependency<MomentumFunction>(Update::Velocity, momentumAlgo_, mc_tvm::Momentum::Output::Velocity);
  addInputDependency<MomentumFunction>(Update::Jacobian, momentumAlgo_, mc_tvm::Momentum::Output::Jacobian);
  addInputDependency<MomentumFunction>(Update::NormalAcceleration, momentumAlgo_,
                                       mc_tvm::Momentum::Output::NormalAcceleration);
  addInputDependency<MomentumFunction>(Update::JDot, momentumAlgo_, mc_tvm::Momentum::Output::JDot);
}

void MomentumFunction::reset()
{
  momentum_ = momentumAlgo_.momentum();
  refVel_.setZero();
  refAccel_.setZero();
}

void MomentumFunction::updateValue()
{
  value_ = (momentumAlgo_.momentum() - momentum_).vector();
}

void MomentumFunction::updateVelocity()
{
  // Giving -refVel as velocity error and not momentumAlgo_.velocity() - refVel_
  // because it's a feedforward (momentum is already a velocity)
  velocity_ = -refVel_;
}

void MomentumFunction::updateJacobian()
{
  splitJacobian(momentumAlgo_.jacobian(), momentumAlgo_.robot().q());
}

void MomentumFunction::updateNormalAcceleration()
{
  normalAcceleration_ = momentumAlgo_.normalAcceleration().vector() - refAccel_;
}

void MomentumFunction::updateJDot()
{
  splitJacobian(momentumAlgo_.JDot(), momentumAlgo_.robot().q());
}

} // namespace mc_tvm
