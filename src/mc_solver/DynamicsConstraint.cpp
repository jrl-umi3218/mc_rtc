#include <mc_solver/DynamicsConstraint.h>

#include <Tasks/Bounds.h>

namespace mc_solver
{

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep)
{
  build_constr(robots, robotIndex, infTorque);
}

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, const std::array<double, 3> & damper, double velocityPercent, bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep, damper, velocityPercent)
{
  build_constr(robots, robotIndex, infTorque);
}

void DynamicsConstraint::build_constr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, bool infTorque)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  std::vector< std::vector<double> > tl = robot.tl();
  std::vector< std::vector<double> > tu = robot.tu();
  if(infTorque)
  {
    for(auto & ti : tl)
    {
      for(auto & t : ti)
      {
        t = -INFINITY;
      }
    }
    for(auto & ti : tu)
    {
      for(auto & t : ti)
      {
        t = INFINITY;
      }
    }
  }
  tasks::TorqueBound tBound(tl, tu);
  if(robot.flexibility().size() != 0)
  {
    std::vector<tasks::qp::SpringJoint> sjList;
    for(const auto & flex : robot.flexibility())
    {
      sjList.push_back(tasks::qp::SpringJoint(flex.jointName, flex.K, flex.C, flex.O));
    }
    motionSpringConstr.reset(new tasks::qp::MotionSpringConstr(robots.mbs(), static_cast<int>(robotIndex), tBound, sjList));
  }
  /*FIXME Implement?
  else if(robot.tlPoly.size() != 0)
  {
  } */
  else
  {
    motionConstr.reset(new tasks::qp::MotionConstr(robots.mbs(), static_cast<int>(robotIndex), tBound));
  }
}

void DynamicsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const
{
  KinematicsConstraint::addToSolver(mbs, solver);
  if(motionConstr)
  {
    motionConstr->addToSolver(mbs, solver);
  }
  if(motionSpringConstr)
  {
    motionSpringConstr->addToSolver(mbs, solver);
  }
}

void DynamicsConstraint::removeFromSolver(tasks::qp::QPSolver & solver) const
{
  KinematicsConstraint::removeFromSolver(solver);
  if(motionConstr)
  {
    motionConstr->removeFromSolver(solver);
  }
  if(motionSpringConstr)
  {
    motionSpringConstr->removeFromSolver(solver);
  }
}

} // namespace mc_solver
