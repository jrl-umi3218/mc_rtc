/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CoMIncPlaneConstr.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/CoMInConvexFunction.h>

#include <Tasks/QPConstr.h>

#include <tvm/task_dynamics/VelocityDamper.h>

#include "utils/jointsToSelector.h"

namespace mc_solver
{

namespace details
{

struct TasksCoMIncPlaneConstr
{
  const mc_rbdyn::Robot & robot_;
  tasks::qp::CoMIncPlaneConstr constraint_;

  TasksCoMIncPlaneConstr(const mc_rbdyn::Robot & robot, double dt)
  : robot_(robot), constraint_(robot.robots().mbs(), static_cast<int>(robot.robotIndex()), dt)
  {
  }
};

struct TVMCoMIncPlaneConstr
{
  mc_tvm::CoMInConvexFunctionPtr function_;
  tvm::task_dynamics::VelocityDamper::Config config_{0.05, 0.01, 0.1, 0.0};
  tvm::TaskWithRequirementsPtr constraint_;

  TVMCoMIncPlaneConstr(const mc_rbdyn::Robot & robot) : function_(std::make_shared<mc_tvm::CoMInConvexFunction>(robot))
  {
  }

  void addToSolver(mc_solver::TVMQPSolver & solver)
  {
    constraint_ = solver.problem().add(function_ >= 0., tvm::task_dynamics::VelocityDamper(solver.dt(), config_),
                                       {tvm::requirements::PriorityLevel(0)});
  }

  void removeFromSolver(mc_solver::TVMQPSolver & solver)
  {
    solver.problem().remove(*constraint_);
    constraint_.reset();
  }

  void setPlanes(mc_solver::TVMQPSolver & solver,
                 const std::vector<mc_rbdyn::Plane> & planes,
                 const std::vector<Eigen::Vector3d> & speeds,
                 const std::vector<Eigen::Vector3d> & normalsDots,
                 tvm::task_dynamics::VelocityDamper::Config config)
  {
    auto isNewConfig = [&]()
    {
      return config_.di_ != config.di_ || config_.ds_ != config.ds_ || config_.xsi_ != config.xsi_
             || config_.xsiOff_ != config.xsiOff_;
    };
    bool needInsertion = constraint_ != nullptr && (planes.size() != function_->planes().size() || isNewConfig());
    config_ = config;
    if(needInsertion) { removeFromSolver(solver); }
    const auto & fn_planes = function_->planes();
    auto update_position = [&, this](size_t i) -> tvm::geometry::Plane &
    {
      auto & plane = planes[i];
      if(i < fn_planes.size())
      {
        auto & out = function_->plane(i);
        out.position(plane.normal, plane.offset);
        return out;
      }
      else
      {
        auto out = std::make_shared<tvm::geometry::Plane>(plane.normal, plane.offset);
        function_->addPlane(out);
        return *out;
      }
    };
    if(speeds.size() != 0 && speeds.size() == planes.size() && normalsDots.size() == planes.size())
    {
      for(size_t i = 0; i < planes.size(); ++i)
      {
        auto & fn_plane = update_position(i);
        fn_plane.velocity(speeds[i], normalsDots[i]);
      }
    }
    else
    {
      for(size_t i = 0; i < planes.size(); ++i) { update_position(i); }
    }
    if(needInsertion) { addToSolver(solver); }
  }
};

} // namespace details

/** Helper to cast the constraint */
static inline mc_rtc::void_ptr_caster<details::TasksCoMIncPlaneConstr> tasks_constraint{};
static inline mc_rtc::void_ptr_caster<details::TVMCoMIncPlaneConstr> tvm_constraint{};

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend,
                                        const mc_rbdyn::Robots & robots,
                                        unsigned int robotIndex,
                                        double dt)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return mc_rtc::make_void_ptr<details::TasksCoMIncPlaneConstr>(robots.robot(robotIndex), dt);
    case QPSolver::Backend::TVM:
      return mc_rtc::make_void_ptr<details::TVMCoMIncPlaneConstr>(robots.robot(robotIndex));
    default:
      mc_rtc::log::error_and_throw("[CoMIncPlaneConstr] Not implemented for solver backend: {}", backend);
  }
}

CoMIncPlaneConstr::CoMIncPlaneConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt)
: constraint_(make_constraint(backend_, robots, robotIndex, dt))
{
}

void CoMIncPlaneConstr::addToSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      tasks_constraint(constraint_)->constraint_.addToSolver(solver.robots().mbs(), tasks_solver(solver).solver());
      break;
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->addToSolver(tvm_solver(solver));
      break;
    default:
      break;
  }
}

void CoMIncPlaneConstr::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      tasks_constraint(constraint_)->constraint_.removeFromSolver(tasks_solver(solver).solver());
      break;
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->removeFromSolver(tvm_solver(solver));
      break;
    default:
      break;
  }
}

void CoMIncPlaneConstr::setActiveJoints(const std::vector<std::string> & joints)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      tasks_constraint(constraint_)->constraint_.selector() =
          jointsToSelector(tasks_constraint(constraint_)->robot_, joints);
      break;
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->function_->selector() =
          jointsToSelector(tvm_constraint(constraint_)->function_->robot(), joints);
      break;
    default:
      break;
  }
}

void CoMIncPlaneConstr::setInactiveJoints(const std::vector<std::string> & joints)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      tasks_constraint(constraint_)->constraint_.selector() =
          jointsToSelector<false>(tasks_constraint(constraint_)->robot_, joints);
      break;
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->function_->selector() =
          jointsToSelector<false>(tvm_constraint(constraint_)->function_->robot(), joints);
      break;
    default:
      break;
  }
}

void CoMIncPlaneConstr::resetActiveJoints()
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      tasks_constraint(constraint_)->constraint_.selector().setOnes();
      break;
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->function_->selector().setOnes();
      break;
    default:
      break;
  }
}

void CoMIncPlaneConstr::setPlanes(QPSolver & solver,
                                  const std::vector<mc_rbdyn::Plane> & planes,
                                  const std::vector<Eigen::Vector3d> & speeds,
                                  const std::vector<Eigen::Vector3d> & normalsDots,
                                  double iDist,
                                  double sDist,
                                  double damping,
                                  double dampingOff)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & constr = tasks_constraint(constraint_)->constraint_;
      auto & qpsolver = tasks_solver(solver);
      constr.reset();
      if(speeds.size() != 0 && normalsDots.size() == speeds.size() && planes.size() == speeds.size())
      {
        for(size_t i = 0; i < planes.size(); ++i)
        {
          if(planes[i].normal.norm() > 0.5)
          {
            constr.addPlane(static_cast<int>(i), planes[i].normal, planes[i].offset, iDist, sDist, damping, speeds[i],
                            normalsDots[i], dampingOff);
          }
        }
      }
      else
      {
        if(speeds.size() != 0 && (normalsDots.size() != speeds.size() || planes.size() != speeds.size()))
        {
          mc_rtc::log::warning(
              "set_planes: speeds size > 0 but different from normalsDots or planes, acting as if speeds "
              "were not provided");
        }
        for(size_t i = 0; i < planes.size(); ++i)
        {
          if(planes[i].normal.norm() > 0.5)
          {
            constr.addPlane(static_cast<int>(i), planes[i].normal, planes[i].offset, iDist, sDist, damping, dampingOff);
          }
        }
      }
      constr.updateNrPlanes();
      qpsolver.updateConstrSize();
      break;
    }
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)
          ->setPlanes(tvm_solver(solver), planes, speeds, normalsDots, {iDist, sDist, damping, dampingOff});
      break;
    default:
      break;
  }
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "CoMIncPlane",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto ret = std::make_shared<mc_solver::CoMIncPlaneConstr>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "CoMIncPlane"), solver.dt());
      return ret;
    });
} // namespace
