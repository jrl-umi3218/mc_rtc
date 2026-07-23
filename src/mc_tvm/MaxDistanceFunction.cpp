/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/MaxDistanceFunction.h>

#include <mc_tvm/Robot.h>
#include <mc_tvm/RobotFrame.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotFrame.h>
#include <mc_rbdyn/SCHAddon.h>

namespace mc_tvm
{

static Convex * max(Convex & c1, Convex & c2)
{
  if(c1.frame().robot().mb().nrDof() < c2.frame().robot().mb().nrDof()) { return &c2; }
  return &c1;
}

MaxDistanceFunction::MaxDistanceFunction(Convex & c1,
                                         Convex & c2,
                                         const Eigen::VectorXd & r1Selector,
                                         const Eigen::VectorXd & r2Selector,
                                         double dt)
: tvm::function::abstract::Function(1), c1_(max(c1, c2)), c2_(c1_ == &c1 ? &c2 : &c1), dt_(dt),
  pair_(c1_->convex().get(), c2_->convex().get())
{
  // clang-format off
  registerUpdates(Update::Value, &MaxDistanceFunction::updateValue,
                  Update::Velocity, &MaxDistanceFunction::updateVelocity,
                  Update::Jacobian, &MaxDistanceFunction::updateJacobian,
                  Update::NormalAcceleration, &MaxDistanceFunction::updateNormalAcceleration);
  // clang-format on

  addOutputDependency<MaxDistanceFunction>(Output::Value, Update::Value);
  addOutputDependency<MaxDistanceFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<MaxDistanceFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<MaxDistanceFunction>(Output::NormalAcceleration, Update::NormalAcceleration);

  addInternalDependency<MaxDistanceFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<MaxDistanceFunction>(Update::Velocity, Update::Jacobian);
  addInternalDependency<MaxDistanceFunction>(Update::NormalAcceleration, Update::Value);

  const Eigen::VectorXd * r1Selector_ = c1_ == &c1 ? &r1Selector : &r2Selector;
  const Eigen::VectorXd * r2Selector_ = c1_ == &c1 ? &r2Selector : &r1Selector;
  auto addConvex = [this](Convex & convex, const Eigen::VectorXd & selector)
  {
    auto & r = convex.frame().robot();
    if(r.mb().nrDof() > 0)
    {
      auto & tvm_robot = r.tvmRobot();
      addInputDependency<MaxDistanceFunction>(Update::Value, convex, Convex::Output::Position);
      addInputDependency<MaxDistanceFunction>(Update::Jacobian, tvm_robot, mc_tvm::Robot::Output::FV);
      addInputDependency<MaxDistanceFunction>(Update::NormalAcceleration, tvm_robot,
                                              mc_tvm::Robot::Output::NormalAcceleration);
      addVariable(tvm_robot.q(), false);
      data_.push_back({Eigen::Vector3d::Zero(), convex.frame().tvm_frame().rbdJacobian(), selector});
    }
    return r.mb().nrDof();
  };
  // By construction c1's robot has more dofs than c2's
  auto maxDof = addConvex(*c1_, *r1Selector_);
  addConvex(*c2_, *r2Selector_);

  fullJac_.resize(1, maxDof);
  distJac_.resize(1, maxDof);
}

void MaxDistanceFunction::updateValue()
{
  double dist = sch::mc_rbdyn::distance(pair_, p1_, p2_);
  if(dist == 0) { dist = sch::epsilon; }
  dist = dist >= 0 ? std::sqrt(dist) : -std::sqrt(-dist);
  normVecDist_ = (p1_ - p2_) / dist;
  if(iter_ == 1) { prevNormVecDist_ = normVecDist_; }
  if(prevIter_ != iter_)
  {
    speedVec_ = (normVecDist_ - prevNormVecDist_) / dt_;
    prevNormVecDist_ = normVecDist_;
    prevIter_ = iter_;
  }
  auto object = std::ref(c1_);
  auto point = std::ref(p1_);
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & d = data_[i];
    d.nearestPoint_ = (sva::PTransformd(point) * object.get()->frame().position().inv()).translation();
    d.jac_.point(d.nearestPoint_);
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
  value_(0) = dist;
}

void MaxDistanceFunction::tick()
{
  iter_++;
}

void MaxDistanceFunction::updateJacobian()
{
  for(int i = 0; i < variables_.numberOfVariables(); ++i) { jacobian_[variables_[i].get()].setZero(); }
  double sign = 1.0;
  auto object = std::ref(c1_);
  auto point = std::ref(p1_);
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & d = data_[i];
    const auto & r = object.get()->frame().robot();
    const auto & tvm_robot = r.tvmRobot();
    const auto & jac = d.jac_.jacobian(r.mb(), r.mbc());
    distJac_.block(0, 0, 1, d.jac_.dof()).noalias() =
        (sign * normVecDist_).transpose() * jac.block(3, 0, 3, d.jac_.dof());
    d.jac_.fullJacobian(r.mb(), distJac_.block(0, 0, 1, d.jac_.dof()), fullJac_);
    if(d.selector_.size() == 0) { jacobian_[tvm_robot.q().get()] += fullJac_.block(0, 0, 1, r.mb().nrDof()); }
    else
    {
      jacobian_[tvm_robot.q().get()] += fullJac_.block(0, 0, 1, r.mb().nrDof()) * d.selector_.asDiagonal();
    }
    sign = -1.0;
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
}

void MaxDistanceFunction::updateVelocity()
{
  velocity_(0) = 0;
  double sign = 1.0;
  auto object = std::ref(c1_);
  auto point = std::ref(p1_);
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & d = data_[i];
    const auto & r = object.get()->frame().robot();
    velocity_(0) += sign * d.jac_.velocity(r.mb(), r.mbc()).linear().dot(normVecDist_);
    sign = -1.0;
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
}

void MaxDistanceFunction::updateNormalAcceleration()
{
  normalAcceleration_(0) = 0;
  double sign = 1.0;
  auto object = std::ref(c1_);
  auto point = std::ref(p1_);
  for(size_t i = 0; i < data_.size(); ++i)
  {
    auto & d = data_[i];
    const auto & r = object.get()->frame().robot();
    const auto & tvm_robot = r.tvmRobot();
    Eigen::Vector3d pNormalAcc = d.jac_.normalAcceleration(r.mb(), r.mbc(), tvm_robot.normalAccB()).linear();
    Eigen::Vector3d pSpeed = d.jac_.velocity(r.mb(), r.mbc()).linear();
    normalAcceleration_(0) += sign * (pNormalAcc.dot(normVecDist_) + pSpeed.dot(speedVec_));
    sign = -1.0;
    object = std::ref(c2_);
    point = std::ref(p2_);
  }
}

} // namespace mc_tvm
