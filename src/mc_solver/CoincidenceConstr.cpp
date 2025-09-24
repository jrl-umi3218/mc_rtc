/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CoincidenceConstr.h>

#include <mc_rtc/logging.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/QPSolver.h>
#include <mc_solver/TasksQPSolver.h>


#include <Tasks/QPCoincidenceConstr.h>
#include <iostream>

#include <mc_rbdyn/Robots.h>

namespace mc_solver
{

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend,
                                        int robot_index,
                                        const std::string & name1,
                                        const std::string & name2,
                                        const Eigen::VectorXd & joints,
                                        const std::string & type)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
    {
      if(type == "6d")
      {
        return mc_rtc::make_void_ptr<tasks::qp::FixedCoincidenceConstr>(robot_index, name1, name2, joints);
      }
      else if(type == "3d")
      {
        return mc_rtc::make_void_ptr<tasks::qp::RotationalCoincidenceConstr>(robot_index, name1, name2, joints);
      }
      else
      {
        mc_rtc::log::error_and_throw("[CoincidenceConstraint] Invalid coincidence type given to constructor");
      }
    }
    case QPSolver::Backend::TVM:
      return {nullptr, nullptr};
    default:
      mc_rtc::log::error_and_throw("[CoincidenceConstraint] Not implemented for solver backend: {}", backend);
  }
}

CoincidenceConstraint::CoincidenceConstraint(const mc_rbdyn::Robots & robots, std::string name1, std::string name2, std::string type, const Eigen::VectorXd & joints, double dt)
: ConstraintSet(), 
  name1_(name1), 
  name2_(name2), 
  type_(type), 
  joints_(joints),
  dt_(dt),
  constraint_(make_constraint(backend_, robots.robotIndex(), name1_, name2_, joints_, type_))
{
}

void CoincidenceConstraint::addToSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::CoincidenceConstr *>(constraint_.get())
          ->addToSolver(solver.robots().mbs(), tasks_solver(solver).solver());
      mc_rtc::log::success("CoincidenceConstraint {} <-> {} added successfully!", name1_, name2_);
      break;
    case QPSolver::Backend::TVM:
      break;
  }
}


void CoincidenceConstraint::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::CoincidenceConstr *>(constraint_.get())
          ->removeFromSolver(tasks_solver(solver).solver());
      mc_rtc::log::info("CoincidenceConstraint {} <-> {} removed", name1_, name2_);
      break;
    case QPSolver::Backend::TVM:
      break;
  }
}

tasks::qp::CoincidenceConstr * CoincidenceConstraint::coincidenceConstr()
{
  if(backend_ != QPSolver::Backend::Tasks)
  {
    mc_rtc::log::error_and_throw("[CoincidenceConstraint] coincidenceConstr() is only usable for the Tasks backend");
  }
  return static_cast<tasks::qp::CoincidenceConstr *>(constraint_.get());
}

} // namespace mc_solver
