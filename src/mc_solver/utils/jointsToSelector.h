#pragma once

#include <mc_rbdyn/Robot.h>

namespace mc_solver
{

template<bool Select = true>
static inline Eigen::VectorXd jointsToSelector(const mc_rbdyn::Robot & robot, const std::vector<std::string> & joints)
{
  if(joints.size() == 0) { return Eigen::VectorXd::Zero(0); }
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
