/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/CoMInConvexFunction.h>

#include <mc_tvm/Robot.h>

namespace mc_tvm
{

CoMInConvexFunction::CoMInConvexFunction(const mc_rbdyn::Robot & robot)
: tvm::function::abstract::Function(0), com_(robot.tvmRobot().comAlgo())
{
  registerUpdates(Update::Value, &CoMInConvexFunction::updateValue, Update::Velocity,
                  &CoMInConvexFunction::updateVelocity, Update::Jacobian, &CoMInConvexFunction::updateJacobian,
                  Update::NormalAcceleration, &CoMInConvexFunction::updateNormalAcceleration);
  addOutputDependency<CoMInConvexFunction>(Output::Value, Update::Value);
  addOutputDependency<CoMInConvexFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<CoMInConvexFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<CoMInConvexFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  addVariable(robot.tvmRobot().q(), false);
  addInputDependency<CoMInConvexFunction>(Update::Value, com_, mc_tvm::CoM::Output::CoM);
  addInputDependency<CoMInConvexFunction>(Update::Velocity, com_, mc_tvm::CoM::Output::Velocity);
  addInputDependency<CoMInConvexFunction>(Update::Jacobian, com_, mc_tvm::CoM::Output::Jacobian);
  addInputDependency<CoMInConvexFunction>(Update::NormalAcceleration, com_, mc_tvm::CoM::Output::NormalAcceleration);
  addInternalDependency<CoMInConvexFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<CoMInConvexFunction>(Update::Velocity, Update::Jacobian);
  addInternalDependency<CoMInConvexFunction>(Update::NormalAcceleration, Update::Velocity);
}

void CoMInConvexFunction::addPlane(tvm::geometry::PlanePtr plane)
{
  planes_.push_back(plane);
  addInputDependency<CoMInConvexFunction>(Update::Value, plane, tvm::geometry::Plane::Output::Position);
  addInputDependency<CoMInConvexFunction>(Update::Velocity, plane, tvm::geometry::Plane::Output::Velocity);
  addInputDependency<CoMInConvexFunction>(Update::NormalAcceleration, plane,
                                          tvm::geometry::Plane::Output::Acceleration);
  resize(static_cast<int>(planes_.size()));
}

void CoMInConvexFunction::reset()
{
  planes_.resize(0);
  resize(0);
}

void CoMInConvexFunction::updateValue()
{
  Eigen::DenseIndex i = 0;
  for(const auto & p : planes_)
  {
    value_(i++) = com_.com().dot(p->normal()) + p->offset();
  }
}

void CoMInConvexFunction::updateVelocity()
{
  Eigen::DenseIndex i = 0;
  for(const auto & p : planes_)
  {
    velocity_(i++) = p->normal().dot(com_.velocity() - p->speed()) + p->normalDot().dot(com_.com() - p->point());
  }
}

void CoMInConvexFunction::updateJacobian()
{
  Eigen::DenseIndex i = 0;
  const Eigen::MatrixXd & jac = com_.jacobian();
  for(const auto & p : planes_)
  {
    jacobian_[com_.robot().q().get()].row(i).noalias() = p->normal().transpose() * jac;
    ++i;
  }
}

void CoMInConvexFunction::updateNormalAcceleration()
{
  Eigen::DenseIndex i = 0;
  for(const auto & p : planes_)
  {
    normalAcceleration_(i++) = p->normal().dot(com_.normalAcceleration() - p->acceleration())
                               + 2 * p->normalDot().dot(com_.velocity() - p->speed())
                               + p->normalDotDot().dot(com_.com() - p->point());
  }
}

} // namespace mc_tvm
