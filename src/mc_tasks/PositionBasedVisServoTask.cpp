/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/PositionBasedVisServoTask.h>

#include <mc_rtc/deprecated.h>

#include <Eigen/Geometry>

namespace mc_tasks
{

PositionBasedVisServoTask::PositionBasedVisServoTask(const std::string & bodyName,
                                                     const sva::PTransformd & X_t_s,
                                                     const sva::PTransformd & X_b_s,
                                                     const mc_rbdyn::Robots & robots,
                                                     unsigned int robotIndex,
                                                     double stiffness,
                                                     double weight)
: TrajectoryTaskGeneric<tasks::qp::PositionBasedVisServoTask>(robots, robotIndex, stiffness, weight), X_t_s_(X_t_s)
{
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, X_t_s, X_b_s);
  type_ = "pbvs";
  name_ = "pbvs_" + robots.robot(robotIndex).name() + "_" + bodyName;
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
  finalize(robots.mbs(), static_cast<int>(rIndex), frame.body(), X_t_s_, frame.X_b_f());
  type_ = "pbvs";
  name_ = "pbvs_" + frame.robot().name() + "_" + frame.name();
}

void PositionBasedVisServoTask::reset()
{
  TrajectoryTaskGeneric::reset();
  X_t_s_ = sva::PTransformd::Identity();
  errorT->error(X_t_s_);
}

void PositionBasedVisServoTask::error(const sva::PTransformd & X_t_s)
{
  X_t_s_ = X_t_s;
  errorT->error(X_t_s_);
}

void PositionBasedVisServoTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  MC_RTC_LOG_HELPER(name_ + "_error", X_t_s_);
  logger.addLogEntry(name_ + "_eval", this, [this]() -> sva::PTransformd {
    Eigen::Vector6d eval = errorT->eval();
    Eigen::Vector3d angleAxis = eval.head(3);
    Eigen::Vector3d axis = angleAxis / angleAxis.norm();
    double angle = angleAxis.dot(axis);
    Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, axis));
    return sva::PTransformd(quat, eval.tail(3));
  });
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "pbvs",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
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
}
