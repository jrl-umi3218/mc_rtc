/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/JointsSelectorFunction.h>

#include <mc_tvm/Robot.h>

namespace mc_tvm
{

std::unique_ptr<JointsSelectorFunction> JointsSelectorFunction::ActiveJoints(
    Function * f,
    const mc_rbdyn::Robot & robot,
    const std::vector<std::string> & activeJoints)
{
  auto & tvm_robot = robot.tvmRobot();
  tvm::VariablePtr usedVariable = nullptr;
  auto useRobotVariable = [&](const tvm::VariablePtr & v)
  {
    if(std::find(f->variables().begin(), f->variables().end(), v) != f->variables().end())
    {
      usedVariable = v;
      return true;
    }
    return false;
  };
  if(!useRobotVariable(tvm_robot.q()) && !useRobotVariable(tvm_robot.qJoints()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Cannot setup joint selector on the provided function as it doesn't use variables of {}", robot.name());
  }

  const auto & mb = robot.mb();

  std::vector<std::string> joints = activeJoints;
  std::sort(joints.begin(), joints.end(),
            [&mb](const std::string & lhs, const std::string & rhs)
            {
              auto lhs_idx = mb.jointIndexByName(lhs);
              auto rhs_idx = mb.jointIndexByName(rhs);
              auto lhs_pos = mb.jointPosInDof(lhs_idx);
              auto rhs_pos = mb.jointPosInDof(rhs_idx);
              return lhs_pos < rhs_pos || (lhs_pos == rhs_pos && mb.joint(lhs_idx).dof() < mb.joint(rhs_idx).dof());
            });

  std::vector<std::pair<Eigen::DenseIndex, Eigen::DenseIndex>> activeIndex;
  Eigen::DenseIndex offset = usedVariable->spaceShift().tSize();
  Eigen::DenseIndex start = offset;
  Eigen::DenseIndex size = 0;
  for(const auto & j : joints)
  {
    auto jIndex = mb.jointIndexByName(j);
    auto jDof = mb.joint(jIndex).dof();
    Eigen::DenseIndex pos = mb.jointPosInDof(jIndex);
    if(pos != start + size)
    {
      if(size != 0) { activeIndex.push_back({start - offset, size}); }
      start = pos;
      size = jDof;
    }
    else
    {
      size += jDof;
    }
  }
  if(size != 0) { activeIndex.push_back({start - offset, size}); }

  return std::unique_ptr<JointsSelectorFunction>(new JointsSelectorFunction(f, usedVariable, activeIndex));
}

std::unique_ptr<JointsSelectorFunction> JointsSelectorFunction::InactiveJoints(
    Function * f,
    const mc_rbdyn::Robot & robot,
    const std::vector<std::string> & inactiveJoints)
{
  auto & tvm_robot = robot.tvmRobot();
  size_t j_idx = 0;
  std::vector<std::string> activeJoints{};
  auto useRobotVariable = [&](const tvm::VariablePtr & v)
  {
    if(std::find(f->variables().begin(), f->variables().end(), v) != f->variables().end()) { return true; }
    return false;
  };
  if(!useRobotVariable(tvm_robot.q()) && useRobotVariable(tvm_robot.qJoints()))
  {
    j_idx = tvm_robot.qFloatingBase()->size() != 0 ? 1 : 0;
  }
  for(; j_idx < robot.mb().joints().size(); ++j_idx)
  {
    const auto & j = robot.mb().joints()[j_idx];
    if(std::find_if(inactiveJoints.begin(), inactiveJoints.end(), [&j](const std::string & s) { return s == j.name(); })
       == inactiveJoints.end())
    {
      activeJoints.push_back(j.name());
    }
  }
  return ActiveJoints(f, robot, activeJoints);
}

JointsSelectorFunction::JointsSelectorFunction(
    Function * f,
    tvm::VariablePtr variable,
    const std::vector<std::pair<Eigen::DenseIndex, Eigen::DenseIndex>> & activeIndex)
: tvm::function::abstract::Function(f->imageSpace()), f_(f), variable_(variable), activeIndex_(activeIndex)
{
  addDirectDependency<JointsSelectorFunction>(Output::Value, *f_, Function::Output::Value);
  addDirectDependency<JointsSelectorFunction>(Output::Velocity, *f_, Function::Output::Velocity);
  addDirectDependency<JointsSelectorFunction>(Output::NormalAcceleration, *f_, Function::Output::NormalAcceleration);

  registerUpdates(Update::Jacobian, &JointsSelectorFunction::updateJacobian);
  registerUpdates(Update::JDot, &JointsSelectorFunction::updateJDot);

  addOutputDependency<JointsSelectorFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<JointsSelectorFunction>(Output::JDot, Update::JDot);

  addInputDependency<JointsSelectorFunction>(Update::Jacobian, *f_, Function::Output::Jacobian);
  addInputDependency<JointsSelectorFunction>(Update::JDot, *f_, Function::Output::JDot);

  addVariable(variable_, f_->linearIn(*variable_));
  jacobian_.at(variable.get()).setZero();
  JDot_.at(variable.get()).setZero();
}

void JointsSelectorFunction::updateJacobian()
{
  const auto & jacIn = f_->jacobian(*variable_);
  for(const auto & p : activeIndex_)
  {
    jacobian_[variable_.get()].middleCols(p.first, p.second) = jacIn.block(0, p.first, jacIn.rows(), p.second);
  }
}

void JointsSelectorFunction::updateJDot()
{
  const auto & JDotIn = f_->jacobian(*variable_);
  for(const auto & p : activeIndex_)
  {
    JDot_[variable_.get()].middleCols(p.first, p.second) = JDotIn.block(0, p.first, JDotIn.rows(), p.second);
  }
}

} // namespace mc_tvm
