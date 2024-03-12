/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/PositionBasedVisServoTask.h>

#include <mc_tvm/PositionBasedVisServoFunction.h>

#include <mc_rtc/deprecated.h>

#include <Eigen/Geometry>

namespace mc_tasks
{

static inline mc_rtc::void_ptr_caster<tasks::qp::PositionBasedVisServoTask> tasks_error{};
static inline mc_rtc::void_ptr_caster<mc_tvm::PositionBasedVisServoFunction> tvm_error{};

PositionBasedVisServoTask::PositionBasedVisServoTask(const std::string & bodyName,
                                                     const sva::PTransformd & X_t_s,
                                                     const sva::PTransformd & X_b_s,
                                                     const mc_rbdyn::Robots & robots,
                                                     unsigned int robotIndex,
                                                     double stiffness,
                                                     double weight)
: TrajectoryTaskGeneric(robots, robotIndex, stiffness, weight), X_t_s_(X_t_s)
{
  name_ = "pbvs_" + robots.robot(robotIndex).name() + "_" + bodyName;
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::PositionBasedVisServoTask>(robots.mbs(), static_cast<int>(rIndex), bodyName,
                                                                     X_t_s, X_b_s);
      break;
    case Backend::TVM:
    {
      const auto & robot = robots.robot(rIndex);
      const auto & bodyFrame = robot.frame(bodyName);
      finalize<Backend::TVM, mc_tvm::PositionBasedVisServoFunction>(
          *robot.makeTemporaryFrame(name_, bodyFrame, X_b_s, true));
      tvm_error(errorT)->error(X_t_s);
      break;
    }
    default:
      mc_rtc::log::error_and_throw("[PBVSTask] Not implemented for solver backend: {}", backend_);
  }
}

PositionBasedVisServoTask::PositionBasedVisServoTask(const std::string & surfaceName,
                                                     const sva::PTransformd & X_t_s,
                                                     const mc_rbdyn::Robots & robots,
                                                     unsigned int robotIndex,
                                                     double stiffness,
                                                     double weight)
: PositionBasedVisServoTask(robots.robot(robotIndex).frame(surfaceName), X_t_s, stiffness, weight)
{
}

PositionBasedVisServoTask::PositionBasedVisServoTask(const mc_rbdyn::RobotFrame & frame,
                                                     const sva::PTransformd & X_t_s,
                                                     double stiffness,
                                                     double weight)
: TrajectoryBase(frame, stiffness, weight), X_t_s_(X_t_s)
{
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::PositionBasedVisServoTask>(robots.mbs(), static_cast<int>(rIndex),
                                                                     frame.body(), X_t_s, frame.X_b_f());
      break;
    case Backend::TVM:
      finalize<Backend::TVM, mc_tvm::PositionBasedVisServoFunction>(frame);
      tvm_error(errorT)->error(X_t_s);
      break;
    default:
      mc_rtc::log::error_and_throw("[PBVSTask] Not implemented for solver backend: {}", backend_);
  }
  type_ = "pbvs";
  name_ = "pbvs_" + frame.robot().name() + "_" + frame.name();
}

void PositionBasedVisServoTask::reset()
{
  TrajectoryTaskGeneric::reset();
  error(sva::PTransformd::Identity());
}

void PositionBasedVisServoTask::error(const sva::PTransformd & X_t_s)
{
  X_t_s_ = X_t_s;
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->error(X_t_s_);
      break;
    case Backend::TVM:
      tvm_error(errorT)->error(X_t_s_);
      break;
    default:
      break;
  }
}

void PositionBasedVisServoTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  MC_RTC_LOG_HELPER(name_ + "_error", X_t_s_);
  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto error = tasks_error(errorT);
      logger.addLogEntry(name_ + "_eval", this,
                         [error]() -> sva::PTransformd
                         {
                           Eigen::Vector6d eval = error->eval();
                           Eigen::Vector3d angleAxis = eval.head(3);
                           Eigen::Vector3d axis = angleAxis / angleAxis.norm();
                           double angle = angleAxis.dot(axis);
                           Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, axis));
                           return sva::PTransformd(quat, eval.tail(3));
                         });
      break;
    }
    case Backend::TVM:
    {
      auto error = tvm_error(errorT);
      logger.addLogEntry(name_ + "_eval", this,
                         [error]() -> sva::PTransformd
                         {
                           Eigen::Vector6d eval = error->value();
                           Eigen::Vector3d angleAxis = eval.head(3);
                           Eigen::Vector3d axis = angleAxis / angleAxis.norm();
                           double angle = angleAxis.dot(axis);
                           Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, axis));
                           return sva::PTransformd(quat, eval.tail(3));
                         });
      break;
    }
    default:
      break;
  }
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "pbvs",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      std::shared_ptr<mc_tasks::PositionBasedVisServoTask> t;
      auto robotIndex = robotIndexFromConfig(config, solver.robots(), "pbvs");
      if(config.has("frame"))
      {
        const auto & robot = solver.robots().robot(robotIndex);
        t = std::make_shared<mc_tasks::PositionBasedVisServoTask>(robot.frame(config("frame")),
                                                                  sva::PTransformd::Identity());
      }
      else if(config.has("surface"))
      {
        mc_rtc::log::deprecated("PositionBasedVisServoTask", "surface", "frame");
        t = std::make_shared<mc_tasks::PositionBasedVisServoTask>(config("surface"), sva::PTransformd::Identity(),
                                                                  solver.robots(), robotIndex);
      }
      else if(config.has("body"))
      {
        mc_rtc::log::deprecated("PositionBasedVisServoTask", "body", "frame");
        t = std::make_shared<mc_tasks::PositionBasedVisServoTask>(config("body"), sva::PTransformd::Identity(),
                                                                  config("X_b_s", sva::PTransformd::Identity()),
                                                                  solver.robots(), robotIndex);
      }
      t->load(solver, config);
      return t;
    });
} // namespace
