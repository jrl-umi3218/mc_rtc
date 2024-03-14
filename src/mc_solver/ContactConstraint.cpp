/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/ContactConstraint.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_rtc/logging.h>

#include <Tasks/QPContactConstr.h>
#include <Tasks/QPMotionConstr.h>

namespace mc_solver
{

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend,
                                        double timeStep,
                                        ContactConstraint::ContactType contactType)
{
  using CType = ContactConstraint::ContactType;
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
    {
      switch(contactType)
      {
        case CType::Acceleration:
          return mc_rtc::make_void_ptr<tasks::qp::ContactAccConstr>();
        case CType::Velocity:
          return mc_rtc::make_void_ptr<tasks::qp::ContactSpeedConstr>(timeStep);
        case CType::Position:
          return mc_rtc::make_void_ptr<tasks::qp::ContactPosConstr>(timeStep);
        default:
          mc_rtc::log::error_and_throw("[ContactConstraint] Invalid contact type given to constructor");
      }
    }
    case QPSolver::Backend::TVM:
      return {nullptr, nullptr};
    default:
      mc_rtc::log::error_and_throw("[ContactConstr] Not implemented for solver backend: {}", backend);
  }
}

ContactConstraint::ContactConstraint(double timeStep, ContactType contactType)
: constraint_(make_constraint(backend_, timeStep, contactType))
{
}

void ContactConstraint::addToSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::ContactConstr *>(constraint_.get())
          ->addToSolver(solver.robots().mbs(), tasks_solver(solver).solver());
      break;
    case QPSolver::Backend::TVM:
      break;
    default:
      break;
  }
}

void ContactConstraint::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      static_cast<tasks::qp::ContactConstr *>(constraint_.get())->removeFromSolver(tasks_solver(solver).solver());
      break;
    case QPSolver::Backend::TVM:
      break;
    default:
      break;
  }
}

tasks::qp::ContactConstr * ContactConstraint::contactConstr()
{
  if(backend_ != QPSolver::Backend::Tasks)
  {
    mc_rtc::log::error_and_throw("[ContactConstraint] contactConstr() is only usable for the Tasks backend");
  }
  return static_cast<tasks::qp::ContactConstr *>(constraint_.get());
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "contact",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      std::string cTypeStr = config("contactType", std::string{"velocity"});
      auto cType = mc_solver::ContactConstraint::Velocity;
      if(cTypeStr == "acceleration") { cType = mc_solver::ContactConstraint::Acceleration; }
      else if(cTypeStr == "position") { cType = mc_solver::ContactConstraint::Position; }
      else if(cTypeStr != "velocity")
      {
        mc_rtc::log::error("Stored contact type for contact constraint not recognized, default to velocity");
        mc_rtc::log::warning("(Read: {}, expect one of: acceleration, velocity, position)", cTypeStr);
      }
      return std::make_shared<mc_solver::ContactConstraint>(solver.dt(), cType);
    });
} // namespace
