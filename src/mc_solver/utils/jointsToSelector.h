#pragma once

#include <mc_rbdyn/Robot.h>

namespace mc_solver
{

/**
 * @brief Returns a joint selector for the given list of active/unactive joints
 *
 * @tparam Select If true, the selector will be set to 1 (active) for the given joints, 0 (unactive) otherwise. An empty
 * vector is treated as no active joints.
 * @tparam Select If false, the selector will be set to 0 (unactive) for the given joints, 1 (active) otherwise. An
 * empty vector is treated as no inactive joints.
 *
 * @return Eigen::VectorXd A joint selector consisting of a vector of size robot.mb().nrDof() with 1 for active joints,
 * 0 otherwise or an empty vector when all joints are active.
 */
template<bool Select = true>
static inline Eigen::VectorXd jointsToSelector(const mc_rbdyn::Robot & robot, const std::vector<std::string> & joints)
{
  if(joints.empty())
  {
    if constexpr(Select)
    { // no joints active
      return Eigen::VectorXd::Zero(robot.mb().nrDof());
    }
    else
    { // all joints active
      return Eigen::VectorXd::Zero(0);
    }
  }
  Eigen::VectorXd ret = [&robot]()
  {
    if constexpr(Select) { return Eigen::VectorXd::Zero(robot.mb().nrDof()); }
    else { return Eigen::VectorXd::Ones(robot.mb().nrDof()); }
  }();
  for(const auto & j : joints)
  {
    auto mbcIndex = robot.jointIndexByName(j);
    auto dofIndex = robot.mb().jointPosInDof(static_cast<int>(mbcIndex));
    const auto & joint = robot.mb().joint(static_cast<int>(mbcIndex));
    if constexpr(Select) { ret.segment(dofIndex, joint.dof()).setOnes(); }
    else { ret.segment(dofIndex, joint.dof()).setZero(); }
  }
  return ret;
}

} // namespace mc_solver
