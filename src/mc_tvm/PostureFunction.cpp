/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/PostureFunction.h>

#include <mc_rbdyn/Robot.h>
#include <mc_tvm/Robot.h>

namespace mc_tvm
{

PostureFunction::PostureFunction(const mc_rbdyn::Robot & robot)
: tvm::function::IdentityFunction(robot.tvmRobot().qJoints()), robot_(robot),
  j0_(robot_.mb().joint(0).type() == rbd::Joint::Free ? 1 : 0), refVel_(Eigen::VectorXd::Zero(size())),
  refAccel_(Eigen::VectorXd::Zero(size()))
{
  // For mbc.jointConfig
  addInputDependency<PostureFunction>(Update::Value, robot.tvmRobot(), mc_tvm::Robot::Output::FK);

  reset();
}

void PostureFunction::reset()
{
  posture_ = robot_.mbc().q;
  refVel_.setZero();
  refAccel_.setZero();
}

void PostureFunction::posture(const std::string & j, const std::vector<double> & q)
{
  if(!robot_.hasJoint(j))
  {
    mc_rtc::log::error("[PostureFunction] No joint named {} in {}", j, robot_.name());
    return;
  }
  auto jIndex = static_cast<size_t>(robot_.mb().jointIndexByName(j));
  if(posture_[jIndex].size() != q.size())
  {
    mc_rtc::log::error("[PostureFunction] Wrong size for input target on joint {}, excepted {} got {}", j,
                       posture_[jIndex].size(), q.size());
    return;
  }
  posture_[static_cast<size_t>(jIndex)] = q;
}

namespace
{
bool isValidPosture(const std::vector<std::vector<double>> & ref, const std::vector<std::vector<double>> & in)
{
  if(ref.size() != in.size())
  {
    return false;
  }
  for(size_t i = 0; i < ref.size(); ++i)
  {
    if(ref[i].size() != in[i].size())
    {
      return false;
    }
  }
  return true;
}
} // namespace

void PostureFunction::posture(const std::vector<std::vector<double>> & p)
{
  if(!isValidPosture(posture_, p))
  {
    mc_rtc::log::error("[PostureFunction] Invalid posture provided for {}", robot_.name());
    return;
  }
  posture_ = p;
}

void PostureFunction::updateValue_()
{
  int pos = 0;
  for(int jI = j0_; jI < robot_.mb().nrJoints(); ++jI)
  {
    auto jIdx = static_cast<size_t>(jI);
    const auto & j = robot_.mb().joint(jI);
    if(j.dof() == 1) // prismatic or revolute
    {
      value_(pos) = robot_.mbc().q[jIdx][0] - posture_[jIdx][0];
      pos++;
    }
    else if(j.dof() == 3) // spherical
    {
      Eigen::Matrix3d ori(
          Eigen::Quaterniond(posture_[jIdx][0], posture_[jIdx][1], posture_[jIdx][2], posture_[jIdx][3]).matrix());
      auto error = sva::rotationError(ori, robot_.mbc().jointConfig[jIdx].rotation());
      value_.segment(pos, 3) = error;
      pos += 3;
    }
  }
}

void PostureFunction::updateVelocity_()
{
  tvm::function::IdentityFunction::updateVelocity_();
  velocity_ -= refVel_;
}

} // namespace mc_tvm
