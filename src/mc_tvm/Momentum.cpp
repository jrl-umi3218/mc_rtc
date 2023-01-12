/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/Momentum.h>

#include <mc_tvm/Robot.h>

namespace mc_tvm
{

Momentum::Momentum(NewMomentumToken, CoM & com) : com_(com), mat_(robot().robot().mb())
{
  // clang-format off
  registerUpdates(
                  Update::Momentum, &Momentum::updateMomentum,
                  Update::Jacobian, &Momentum::updateJacobian,
                  Update::NormalAcceleration, &Momentum::updateNormalAcceleration,
                  Update::JDot, &Momentum::updateJDot);
  // clang-format off

  addOutputDependency(Output::Momentum, Update::Momentum);
  addInputDependency(Update::Momentum, robot(), Robot::Output::FK);
  addInputDependency(Update::Momentum, com_, CoM::Output::CoM);

  addOutputDependency(Output::Jacobian, Update::Jacobian);
  addInputDependency(Update::Jacobian, robot(), Robot::Output::FV);
  addInputDependency(Update::Jacobian, com_, CoM::Output::CoM);

  addOutputDependency(Output::NormalAcceleration, Update::NormalAcceleration);
  addInputDependency(Update::NormalAcceleration, robot(), Robot::Output::NormalAcceleration);
  addInputDependency(Update::NormalAcceleration, com_, CoM::Output::Velocity);

  addOutputDependency(Output::JDot, Update::JDot);
  addInputDependency(Update::JDot, robot(), Robot::Output::FV);
  addInputDependency(Update::JDot, com_, CoM::Output::Velocity);

  addInternalDependency(Update::NormalAcceleration, Update::Jacobian);

  updateMomentum();
  updateJacobian();
  updateNormalAcceleration();
  updateJDot();
}

void Momentum::updateMomentum()
{
  const auto & r = robot().robot();
  momentum_ = rbd::computeCentroidalMomentum(r.mb(), r.mbc(), com_.com());
}

void Momentum::updateNormalAcceleration()
{
  const auto & r = robot().robot();
  normalAcceleration_ = mat_.normalMomentumDot(r.mb(), r.mbc(), com_.com(), com_.velocity());
}

void Momentum::updateJacobian()
{
  const auto & r = robot().robot();
  mat_.computeMatrix(r.mb(), r.mbc(), com_.com());
}

void Momentum::updateJDot()
{
  const auto & r = robot().robot();
  mat_.computeMatrixDot(r.mb(), r.mbc(), com_.com(), com_.velocity());
}

} // namespace mc_tvm
