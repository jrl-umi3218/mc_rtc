/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/BoundedSpeedConstr.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/FrameVelocity.h>

#include <mc_rtc/logging.h>

#include <Tasks/QPConstr.h>

#include <tvm/task_dynamics/Proportional.h>

namespace mc_solver
{

namespace details
{

struct TVMBoundedSpeedConstr
{
  struct BoundedSpeedData
  {
    BoundedSpeedData(mc_tvm::FrameVelocityPtr fn,
                     const Eigen::Vector6d & lowerSpeed,
                     const Eigen::Vector6d & upperSpeed)
    : fn(fn), lowerSpeed(lowerSpeed), upperSpeed(upperSpeed)
    {
    }

    mc_tvm::FrameVelocityPtr fn;
    Eigen::Vector6d lowerSpeed;
    Eigen::Vector6d upperSpeed;
    tvm::TaskWithRequirementsPtr task;
  };
  std::vector<BoundedSpeedData> data_;

  std::vector<BoundedSpeedData>::iterator getData(const mc_rbdyn::RobotFrame & frame)
  {
    return std::find_if(
        data_.begin(), data_.end(), [&](const auto & d)
        { return d.fn->frame().name() == frame.name() && d.fn->frame().robot().name() == frame.robot().name(); });
  }

  void addBoundedSpeed(TVMQPSolver * solver,
                       const mc_rbdyn::RobotFrame & frame,
                       const Eigen::Vector6d & dof,
                       const Eigen::Vector6d & lowerSpeed,
                       const Eigen::Vector6d & upperSpeed)
  {
    auto it = getData(frame);
    if(it != data_.end())
    {
      it->lowerSpeed = lowerSpeed;
      it->upperSpeed = upperSpeed;
      it->fn->dof(dof);
      if(solver)
      {
        removeBoundedSpeed(*solver, *it);
        addBoundedSpeed(*solver, *it);
      }
    }
    else
    {
      data_.push_back({std::make_shared<mc_tvm::FrameVelocity>(frame, dof), lowerSpeed, upperSpeed});
      if(solver) { addBoundedSpeed(*solver, data_.back()); }
    }
  }

  bool removeBoundedSpeed(TVMQPSolver * solver, const std::string & frame)
  {
    bool r = false;
    for(auto it = data_.begin(); it != data_.end();)
    {
      if(it->fn->frame().name() == frame)
      {
        if(solver) { removeBoundedSpeed(*solver, *it); }
        it = data_.erase(it);
        r = true;
      }
      else { ++it; }
    }
    return r;
  }

  void addToSolver(TVMQPSolver & solver)
  {
    for(auto & d : data_) { addBoundedSpeed(solver, d); }
  }

  void removeFromSolver(TVMQPSolver & solver)
  {
    for(auto & d : data_) { removeBoundedSpeed(solver, d); }
  }

  void reset(TVMQPSolver * solver)
  {
    if(solver) { removeFromSolver(*solver); }
    data_.clear();
  }

private:
  void addBoundedSpeed(TVMQPSolver & solver, BoundedSpeedData & data)
  {
    auto lower = data.fn->dof().cwiseProduct(data.lowerSpeed);
    auto upper = data.fn->dof().cwiseProduct(data.upperSpeed);
    data.task = solver.problem().add(lower <= data.fn <= upper, tvm::task_dynamics::Proportional(1 / solver.dt()),
                                     {tvm::requirements::PriorityLevel(0)});
  }

  void removeBoundedSpeed(TVMQPSolver & solver, BoundedSpeedData & data)
  {
    solver.problem().remove(*data.task);
    data.task.reset();
  }
};

} // namespace details

/** Helper to cast the constraint */
static inline mc_rtc::void_ptr_caster<tasks::qp::BoundedSpeedConstr> tasks_constraint{};
static inline mc_rtc::void_ptr_caster<details::TVMBoundedSpeedConstr> tvm_constraint{};

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend,
                                        const mc_rbdyn::Robots & robots,
                                        unsigned int robotIndex,
                                        double dt)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return mc_rtc::make_void_ptr<tasks::qp::BoundedSpeedConstr>(robots.mbs(), static_cast<int>(robotIndex), dt);
    case QPSolver::Backend::TVM:
      return mc_rtc::make_void_ptr<details::TVMBoundedSpeedConstr>();
    default:
      mc_rtc::log::error_and_throw("[BoundedSpeedConstr] Not implemented for solver backend: {}", backend);
  }
}

BoundedSpeedConstr::BoundedSpeedConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt)
: constraint_(make_constraint(backend_, robots, robotIndex, dt)), robotIndex(robotIndex)
{
}

void BoundedSpeedConstr::addToSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & qpsolver = tasks_solver(solver).solver();
      tasks_constraint(constraint_)->addToSolver(solver.robots().mbs(), qpsolver);
      break;
    }
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->addToSolver(tvm_solver(solver));
      break;
    default:
      break;
  }
}

void BoundedSpeedConstr::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      tasks_constraint(constraint_)->removeFromSolver(tasks_solver(solver).solver());
      break;
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->removeFromSolver(tvm_solver(solver));
      break;
    default:
      break;
  }
}

size_t BoundedSpeedConstr::nrBoundedSpeeds() const
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      return tasks_constraint(constraint_)->nrBoundedSpeeds();
    case QPSolver::Backend::TVM:
      return tvm_constraint(constraint_)->data_.size();
    default:
      return 0;
  }
}

void BoundedSpeedConstr::addBoundedSpeed(QPSolver & solver,
                                         const std::string & bodyName,
                                         const Eigen::Vector3d & bodyPoint,
                                         const Eigen::MatrixXd & dof,
                                         const Eigen::VectorXd & lowerSpeed,
                                         const Eigen::VectorXd & upperSpeed)
{
  const auto & robot = solver.robots().robot(robotIndex);
  if(robot.hasBody(bodyName))
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto & qpsolver = tasks_solver(solver);
        auto & constr = *tasks_constraint(constraint_);
        constr.addBoundedSpeed(solver.robots().mbs(), bodyName, bodyPoint, dof, lowerSpeed, upperSpeed);
        constr.updateBoundedSpeeds();
        qpsolver.updateConstrSize();
        break;
      }
      case QPSolver::Backend::TVM:
      {
        addBoundedSpeed(solver,
                        *robot.makeTemporaryFrame(fmt::format("BoundedSpeedTempFrame_{}", bodyName),
                                                  robot.frame(bodyName), {bodyPoint}, true),
                        dof, lowerSpeed, upperSpeed);
        break;
      }
      default:
        break;
    }
  }
  else
  {
    mc_rtc::log::error("Could not add bounded speed constraint for body {} since it does not exist in robot {}",
                       bodyName, robot.name());
  }
}

void BoundedSpeedConstr::addBoundedSpeed(QPSolver & solver,
                                         const mc_rbdyn::RobotFrame & frame,
                                         const Eigen::MatrixXd & dof,
                                         const Eigen::VectorXd & lowerSpeed,
                                         const Eigen::VectorXd & upperSpeed)
{
  if(frame.robot().robotIndex() != robotIndex)
  {
    mc_rtc::log::error(
        "Could not add bounded speed constraint for frame {} since it belongs to {} while this constraint acts on {}",
        frame.name(), frame.robot().name(), solver.robot(robotIndex).name());
    return;
  }
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & constr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      constr.addBoundedSpeed(solver.robots().mbs(), frame.body(), Eigen::Vector3d::Zero(), dof * frame.X_b_f().matrix(),
                             lowerSpeed, upperSpeed);
      constr.updateBoundedSpeeds();
      qpsolver.updateConstrSize();
      break;
    }
    case QPSolver::Backend::TVM:
    {
      if(lowerSpeed.size() != 6 || upperSpeed.size() != 6)
      {
        mc_rtc::log::error_and_throw("[BoundedSpeedConstr] In TVM backend you have to specify the 6D speed but "
                                     "lowerSpeed size ({}) or upperSpeed size ({}) is wrong",
                                     lowerSpeed.size(), upperSpeed.size());
      }
      if(dof.rows() != 6 || dof.cols() != 6)
      {
        mc_rtc::log::error_and_throw("[BoundedSpeedConstr] In TVM backend, dof must be a 6x6 matrix but got {}x{}",
                                     dof.rows(), dof.cols());
      }
      tvm_constraint(constraint_)
          ->addBoundedSpeed(inSolver_ ? &tvm_solver(solver) : nullptr, frame, dof.diagonal(), lowerSpeed, upperSpeed);
      break;
    }
    default:
      break;
  }
}

bool BoundedSpeedConstr::removeBoundedSpeed(QPSolver & solver, const std::string & frameName)
{
  bool r = false;
  if(solver.robots().robot(robotIndex).hasFrame(frameName))
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto & constr = *tasks_constraint(constraint_);
        auto & qpsolver = tasks_solver(solver);
        r = constr.removeBoundedSpeed(solver.robots().robot(robotIndex).frame(frameName).body());
        constr.updateBoundedSpeeds();
        qpsolver.updateConstrSize();
        break;
      }
      case QPSolver::Backend::TVM:
      {
        return tvm_constraint(constraint_)->removeBoundedSpeed(inSolver_ ? &tvm_solver(solver) : nullptr, frameName);
      }
      default:
        break;
    }
  }
  else
  {
    mc_rtc::log::error("Could not remove bounded speed constraint for frame {} since it does not exist in robot {}",
                       frameName, solver.robots().robot(robotIndex).name());
  }
  return r;
}

void BoundedSpeedConstr::reset(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & constr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      constr.resetBoundedSpeeds();
      constr.updateBoundedSpeeds();
      qpsolver.updateConstrSize();
      break;
    }
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->reset(inSolver_ ? &tvm_solver(solver) : nullptr);
      break;
    default:
      break;
  }
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "boundedSpeed",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const auto & robot = robotFromConfig(config, solver.robots(), "boundedSpeed");
      auto ret = std::make_shared<mc_solver::BoundedSpeedConstr>(solver.robots(), robot.robotIndex(), solver.dt());
      if(config.has("constraints"))
      {
        for(const auto & c : config("constraints"))
        {
          Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
          if(c.has("dof"))
          {
            const auto & c_dof = c("dof");
            if(c_dof.size() == 6)
            {
              Eigen::Vector6d v = c_dof;
              dof = v.asDiagonal();
            }
            else { dof = c_dof; }
          }
          Eigen::VectorXd lowerSpeed;
          Eigen::VectorXd upperSpeed = [&]() -> Eigen::VectorXd
          {
            if(c.has("speed"))
            {
              lowerSpeed = c("speed");
              return lowerSpeed;
            }
            lowerSpeed = c("lowerSpeed");
            return c("upperSpeed");
          }();
          if(lowerSpeed.size() != upperSpeed.size())
          {
            mc_rtc::log::error_and_throw("lowerSpeed size ({}) must match upperSpeed size ({})", lowerSpeed.size(),
                                         upperSpeed.size());
          }
          if(lowerSpeed.size() != dof.rows())
          {
            mc_rtc::log::error_and_throw("dof rows ({}) must match speed size ({})", dof.rows(), lowerSpeed.size());
          }
          if(c.has("body"))
          {
            ret->addBoundedSpeed(solver, c("body"), c("bodyPoint", Eigen::Vector3d::Zero().eval()), dof, lowerSpeed,
                                 upperSpeed);
          }
          else { ret->addBoundedSpeed(solver, robot.frame(c("frame")), dof, lowerSpeed, upperSpeed); }
        }
      }
      return ret;
    });
} // namespace
