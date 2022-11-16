/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/DynamicsConstraint.h>

#include <Tasks/Bounds.h>

namespace mc_solver
{

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots,
                                       unsigned int robotIndex,
                                       double timeStep,
                                       bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep), inSolver_(false), robotIndex_(robotIndex)
{
  build_constr(robots, robotIndex, infTorque, timeStep);
}

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots,
                                       unsigned int robotIndex,
                                       double timeStep,
                                       const std::array<double, 3> & damper,
                                       double velocityPercent,
                                       bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep, damper, velocityPercent), inSolver_(false), robotIndex_(robotIndex)
{
  build_constr(robots, robotIndex, infTorque, timeStep);
}

void DynamicsConstraint::build_constr(const mc_rbdyn::Robots & robots,
                                      unsigned int robotIndex,
                                      bool infTorque,
                                      double timeStep)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  std::vector<std::vector<double>> tl = robot.tl();
  std::vector<std::vector<double>> tu = robot.tu();
  std::vector<std::vector<double>> tdl = robot.tdl();
  std::vector<std::vector<double>> tdu = robot.tdu();
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
    for(auto & tdi : tdl)
    {
      for(auto & td : tdi)
      {
        td = -INFINITY;
      }
    }
    for(auto & tdi : tdu)
    {
      for(auto & td : tdi)
      {
        td = INFINITY;
      }
    }
  }
  tasks::TorqueBound tBound(tl, tu);
  tasks::TorqueDBound tDBound(tdl, tdu);
  if(robot.flexibility().size() != 0)
  {
    std::vector<tasks::qp::SpringJoint> sjList;
    for(const auto & flex : robot.flexibility())
    {
      sjList.push_back(tasks::qp::SpringJoint(flex.jointName, flex.K, flex.C, flex.O));
    }
    motionConstr.reset(new tasks::qp::MotionSpringConstr(robots.mbs(), static_cast<int>(robotIndex), tBound, tDBound,
                                                         timeStep, sjList));
  }
  else
  {
    motionConstr.reset(
        new tasks::qp::MotionConstr(robots.mbs(), static_cast<int>(robotIndex), tBound, tDBound, timeStep));
  }
}

void DynamicsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver)
{
  if(!inSolver_)
  {
    KinematicsConstraint::addToSolver(mbs, solver);
    motionConstr->addToSolver(mbs, solver);
    inSolver_ = true;
  }
}

void DynamicsConstraint::removeFromSolver(tasks::qp::QPSolver & solver)
{
  if(inSolver_)
  {
    KinematicsConstraint::removeFromSolver(solver);
    motionConstr->removeFromSolver(solver);
    inSolver_ = false;
  }
}

} // namespace mc_solver

namespace
{

mc_solver::ConstraintSetPtr load_kin_constr(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "kinematics");
  if(config.has("damper"))
  {
    return std::make_shared<mc_solver::KinematicsConstraint>(solver.robots(), robotIndex, solver.dt(), config("damper"),
                                                             config("velocityPercent", 0.5));
  }
  else
  {
    return std::make_shared<mc_solver::KinematicsConstraint>(solver.robots(), robotIndex, solver.dt());
  }
}

mc_solver::ConstraintSetPtr load_dyn_constr(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "dynamics");
  if(config.has("damper"))
  {
    return std::make_shared<mc_solver::DynamicsConstraint>(solver.robots(), robotIndex, solver.dt(), config("damper"),
                                                           config("velocityPercent", 0.5), config("infTorque", false));
  }
  else
  {
    return std::make_shared<mc_solver::DynamicsConstraint>(solver.robots(), robotIndex, solver.dt(),
                                                           config("infTorque", false));
  }
}

static auto kin_registered = mc_solver::ConstraintSetLoader::register_load_function("kinematics", &load_kin_constr);
static auto dyn_registered = mc_solver::ConstraintSetLoader::register_load_function("dynamics", &load_dyn_constr);

} // namespace
