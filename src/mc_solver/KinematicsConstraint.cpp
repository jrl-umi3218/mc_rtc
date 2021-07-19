/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/KinematicsConstraint.h>

#include <Tasks/Bounds.h>

#include <array>

namespace mc_solver
{

KinematicsConstraint::KinematicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  tasks::QBound qBound(robot.ql(), robot.qu());
  tasks::AlphaDBound aDBound(robot.al(), robot.au());
  tasks::AlphaDDBound aDDBound(robot.jl(), robot.ju());
  jointLimitsConstr.reset(new tasks::qp::JointLimitsConstr(robots.mbs(), static_cast<int>(robotIndex), qBound, aDBound,
                                                           aDDBound, timeStep));
}

KinematicsConstraint::KinematicsConstraint(const mc_rbdyn::Robots & robots,
                                           unsigned int robotIndex,
                                           double timeStep,
                                           const std::array<double, 3> & damper,
                                           double velocityPercent)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  tasks::QBound qBound(robot.ql(), robot.qu());
  double percentInter = damper[0];
  double percentSecur = damper[1];
  double offset = damper[2];

  std::vector<std::vector<double>> vl = robot.vl();
  std::vector<std::vector<double>> vu = robot.vu();
  std::vector<std::vector<double>> al = robot.al();
  std::vector<std::vector<double>> au = robot.au();
  std::vector<std::vector<double>> jl = robot.jl();
  std::vector<std::vector<double>> ju = robot.ju();
  for(auto & vi : vl)
  {
    for(auto & v : vi)
    {
      v = v * velocityPercent;
    }
  }
  for(auto & vi : vu)
  {
    for(auto & v : vi)
    {
      v = v * velocityPercent;
    }
  }
  tasks::AlphaBound alphaBound(vl, vu);
  tasks::AlphaDBound alphaDBound(al, au);
  tasks::AlphaDDBound alphaDDBound(jl, ju);

  damperJointLimitsConstr.reset(new tasks::qp::DamperJointLimitsConstr(robots.mbs(), static_cast<int>(robotIndex),
                                                                       qBound, alphaBound, alphaDBound, alphaDDBound,
                                                                       percentInter, percentSecur, offset, timeStep));
}

void KinematicsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver)
{
  if(damperJointLimitsConstr)
  {
    damperJointLimitsConstr->addToSolver(mbs, solver);
  }
  if(jointLimitsConstr)
  {
    jointLimitsConstr->addToSolver(mbs, solver);
  }
}

void KinematicsConstraint::removeFromSolver(tasks::qp::QPSolver & solver)
{
  if(damperJointLimitsConstr)
  {
    damperJointLimitsConstr->removeFromSolver(solver);
  }
  if(jointLimitsConstr)
  {
    jointLimitsConstr->removeFromSolver(solver);
  }
}

} // namespace mc_solver
