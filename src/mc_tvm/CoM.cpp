/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/CoM.h>

#include <mc_tvm/Robot.h>

namespace mc_tvm
{

CoM::CoM(NewCoMToken, Robot & robot) : robot_(robot), jac_(robot_.robot().mb())
{
  // clang-format off
  registerUpdates(
                  Update::CoM, &CoM::updateCoM,
                  Update::Jacobian, &CoM::updateJacobian,
                  Update::Velocity, &CoM::updateVelocity,
                  Update::NormalAcceleration, &CoM::updateNormalAcceleration,
                  Update::Acceleration, &CoM::updateAcceleration,
                  Update::JDot, &CoM::updateJDot);
  // clang-format off

  addOutputDependency(Output::CoM, Update::CoM);
  addInputDependency(Update::CoM, robot_, Robot::Output::FK);

  addOutputDependency(Output::Jacobian, Update::Jacobian);
  addInputDependency(Update::Jacobian, robot_, Robot::Output::FV);

  addOutputDependency(Output::Velocity, Update::Velocity);
  addInputDependency(Update::Velocity, robot_, Robot::Output::FV);

  addOutputDependency(Output::NormalAcceleration, Update::NormalAcceleration);
  addInputDependency(Update::NormalAcceleration, robot_, Robot::Output::NormalAcceleration);

  addOutputDependency(Output::Acceleration, Update::Acceleration);
  addInputDependency(Update::Acceleration, robot_, Robot::Output::FA);

  addOutputDependency(Output::JDot, Update::JDot);
  addInputDependency(Update::JDot, robot_, Robot::Output::FV);

  addInternalDependency(Update::Velocity, Update::Jacobian);
  addInternalDependency(Update::NormalAcceleration, Update::Jacobian);
}

void CoM::updateCoM()
{
  const auto & r = robot().robot();
  if(r.mass() > 0)
  {
    com_ = rbd::computeCoM(r.mb(), r.mbc());
  }
  else
  {
    com_ = r.posW().translation();
  }
}

void CoM::updateVelocity()
{
  const auto & r = robot().robot();
  velocity_ = jac_.velocity(r.mb(), r.mbc());
}

void CoM::updateNormalAcceleration()
{
  const auto & r = robot().robot();
  normalAcceleration_ = jac_.normalAcceleration(r.mb(), r.mbc(), robot_.normalAccB());
}

void CoM::updateAcceleration()
{
  const auto & r = robot().robot();
  if(r.mass() > 0)
  {
    acceleration_ = rbd::computeCoMAcceleration(r.mb(), r.mbc());
  }
  else
  {
    acceleration_ = Eigen::Vector3d::Zero();
  }
}

void CoM::updateJacobian()
{
  const auto & r = robot().robot();
  jac_.jacobian(r.mb(), r.mbc());
}

void CoM::updateJDot()
{
  const auto & r = robot().robot();
  jac_.jacobianDot(r.mb(), r.mbc());
}

} // namespace mc_tvm
