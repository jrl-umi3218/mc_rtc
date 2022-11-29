/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/DynamicsConstraint.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TasksQPSolver.h>

#include <Tasks/Bounds.h>
#include <Tasks/QPMotionConstr.h>

namespace mc_solver
{

static mc_rtc::void_ptr initialize_tasks(const mc_rbdyn::Robots & robots,
                                         unsigned int robotIndex,
                                         double timeStep,
                                         bool infTorque)
{
  const auto & robot = robots.robot(robotIndex);
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
    return mc_rtc::make_void_ptr<tasks::qp::MotionSpringConstr>(robots.mbs(), static_cast<int>(robotIndex), tBound,
                                                                tDBound, timeStep, sjList);
  }
  else
  {
    return mc_rtc::make_void_ptr<tasks::qp::MotionConstr>(robots.mbs(), static_cast<int>(robotIndex), tBound, tDBound,
                                                          timeStep);
  }
}

static mc_rtc::void_ptr initialize(QPSolver::Backend backend,
                                   const mc_rbdyn::Robots & robots,
                                   unsigned int robotIndex,
                                   double timeStep,
                                   bool infTorque)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return initialize_tasks(robots, robotIndex, timeStep, infTorque);
    default:
      mc_rtc::log::error_and_throw("[DynamicsConstraint] Not implemented for solver backend: {}", backend);
  }
}

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots,
                                       unsigned int robotIndex,
                                       double timeStep,
                                       bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep),
  motion_constr_(initialize(backend_, robots, robotIndex, timeStep, infTorque)), robotIndex_(robotIndex)
{
}

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots,
                                       unsigned int robotIndex,
                                       double timeStep,
                                       const std::array<double, 3> & damper,
                                       double velocityPercent,
                                       bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep, damper, velocityPercent),
  motion_constr_(initialize(backend_, robots, robotIndex, timeStep, infTorque)), robotIndex_(robotIndex)
{
}

void DynamicsConstraint::addToSolverImpl(QPSolver & solver)
{
  KinematicsConstraint::addToSolverImpl(solver);
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::MotionConstr *>(motion_constr_.get())
          ->addToSolver(solver.robots().mbs(), static_cast<TasksQPSolver &>(solver).solver());
      break;
    default:
      break;
  }
}

void DynamicsConstraint::removeFromSolverImpl(QPSolver & solver)
{
  KinematicsConstraint::removeFromSolverImpl(solver);
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::MotionConstr *>(motion_constr_.get())
          ->removeFromSolver(static_cast<TasksQPSolver &>(solver).solver());
      break;
    default:
      break;
  }
}

namespace
{

void fillJointTorqueImpl(tasks::qp::MotionConstr & motionConstr,
                         const tasks::qp::QPSolver & solver,
                         std::vector<std::vector<double>> & jointTorque)
{
  motionConstr.computeTorque(solver.alphaDVec(), solver.lambdaVec());
  rbd::vectorToParam(motionConstr.torque(), jointTorque);
}

} // namespace

void DynamicsConstraint::fillJointTorque(QPSolver & solver) const
{
  switch(solver.backend())
  {
    case QPSolver::Backend::Tasks:
      fillJointTorqueImpl(*static_cast<tasks::qp::MotionConstr *>(motion_constr_.get()),
                          static_cast<const TasksQPSolver &>(solver).solver(),
                          solver.robots().robot(robotIndex_).mbc().jointTorque);
      break;
    default:
      break;
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
