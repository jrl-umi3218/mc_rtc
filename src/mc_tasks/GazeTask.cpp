/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/GazeTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_tvm/GazeFunction.h>

#include <mc_rbdyn/configuration_io.h>

namespace mc_tasks
{

namespace
{

void check_parameters(const mc_rbdyn::Robots & robots, unsigned int robotIndex, const std::string & bodyName)
{
  if(robotIndex >= robots.size())
  {
    mc_rtc::log::error_and_throw("[mc_tasks::GazeTask] No robot with index {}, robots.size() {}", robotIndex,
                                 robots.size());
  }
  if(!robots.robot(robotIndex).hasBody(bodyName))
  {
    mc_rtc::log::error_and_throw("[mc_tasks::GazeTask] No body named {} in {}", bodyName,
                                 robots.robot(robotIndex).name());
  }
}

} // namespace

static inline mc_rtc::void_ptr_caster<tasks::qp::GazeTask> tasks_error{};
static inline mc_rtc::void_ptr_caster<mc_tvm::GazeFunction> tvm_error{};

GazeTask::GazeTask(const std::string & bodyName,
                   const Eigen::Vector2d & point2d,
                   double depthEstimate,
                   const sva::PTransformd & X_b_gaze,
                   const mc_rbdyn::Robots & robots,
                   unsigned int robotIndex,
                   double stiffness,
                   double weight)
: TrajectoryTaskGeneric(robots, robotIndex, stiffness, weight)
{
  check_parameters(robots, robotIndex, bodyName);
  type_ = "gaze";
  name_ = "gaze_" + robots.robot(robotIndex).name() + "_" + bodyName;
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::GazeTask>(robots.mbs(), static_cast<int>(rIndex), bodyName, point2d,
                                                    depthEstimate, X_b_gaze);
      break;
    case Backend::TVM:
    {
      auto & robot = robots.robot(robotIndex);
      finalize<Backend::TVM, mc_tvm::GazeFunction>(
          *robot.makeTemporaryFrame(name_, robot.frame(bodyName), X_b_gaze, true));
      tvm_error(errorT)->estimate(point2d, depthEstimate);
      break;
    }
    default:
      mc_rtc::log::error_and_throw("[GazeTask] Not implemented for solver backend: {}", backend_);
  }
}

GazeTask::GazeTask(const std::string & bodyName,
                   const Eigen::Vector3d & point3d,
                   const sva::PTransformd & X_b_gaze,
                   const mc_rbdyn::Robots & robots,
                   unsigned int robotIndex,
                   double stiffness,
                   double weight)
: TrajectoryTaskGeneric(robots, robotIndex, stiffness, weight)
{
  check_parameters(robots, robotIndex, bodyName);
  if(point3d.z() <= 0)
  {
    mc_rtc::log::error_and_throw<std::logic_error>(
        "[mc_tasks::GazeTask] Expects the depth estimate to be >0, provided {}", point3d.z());
  }
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::GazeTask>(robots.mbs(), static_cast<int>(rIndex), bodyName, point3d,
                                                    X_b_gaze);
      break;
    case Backend::TVM:
    {
      auto & robot = robots.robot(robotIndex);
      finalize<Backend::TVM, mc_tvm::GazeFunction>(
          *robot.makeTemporaryFrame(name_, robot.frame(bodyName), X_b_gaze, true));
      tvm_error(errorT)->estimate(point3d);
      break;
    }
    default:
      mc_rtc::log::error_and_throw("[GazeTask] Not implemented for solver backend: {}", backend_);
  }
  type_ = "gaze";
  name_ = "gaze_" + robots.robot(robotIndex).name() + "_" + bodyName;
}

GazeTask::GazeTask(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight, const Eigen::Vector3d & error)
: TrajectoryTaskGeneric(frame, stiffness, weight)
{
  if(error.z() <= 0)
  {
    mc_rtc::log::error_and_throw<std::logic_error>(
        "[mc_tasks::GazeTask] Expects the depth estimate to be >0, provided {}", error.z());
  }
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::GazeTask>(robots.mbs(), static_cast<int>(rIndex), frame.body(), error,
                                                    frame.X_b_f());
      break;
    case Backend::TVM:
      finalize<Backend::TVM, mc_tvm::GazeFunction>(frame);
      tvm_error(errorT)->estimate(error);
      break;
    default:
      mc_rtc::log::error_and_throw("[GazeTask] Not implemented for solver backend: {}", backend_);
  }
  type_ = "gaze";
  name_ = fmt::format("{}_{}_{}", type_, frame.robot().name(), frame.name());
}

void GazeTask::reset()
{
  TrajectoryTaskGeneric::reset();
  error(Eigen::Vector2d::Zero().eval(), Eigen::Vector2d::Zero());
}

void GazeTask::error(const Eigen::Vector2d & point2d, const Eigen::Vector2d & point2d_ref)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->error(point2d, point2d_ref);
      break;
    case Backend::TVM:
      tvm_error(errorT)->estimate(point2d);
      tvm_error(errorT)->target(point2d_ref);
      break;
    default:
      break;
  }
}

void GazeTask::error(const Eigen::Vector3d & point3d, const Eigen::Vector2d & point2d_ref)
{
  if(point3d.z() > 0)
  {
    switch(backend_)
    {
      case Backend::Tasks:
        tasks_error(errorT)->error(point3d, point2d_ref);
        break;
      case Backend::TVM:
        tvm_error(errorT)->estimate(point3d);
        tvm_error(errorT)->target(point2d_ref);
        break;
      default:
        break;
    }
  }
  else
  {
    mc_rtc::log::warning(
        "GazeTask expects the depth estimate to be >0, provided {}: ignoring error update for this iteration",
        point3d.z());
  }
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "gaze",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto t = [&]()
      {
        if(config.has("body"))
        {
          return std::make_shared<mc_tasks::GazeTask>(config("body"), Eigen::Vector3d{0, 0, 1},
                                                      config("X_b_gaze", sva::PTransformd::Identity()), solver.robots(),
                                                      robotIndexFromConfig(config, solver.robots(), "gaze"));
        }
        else
        {
          const auto & robot = robotFromConfig(config, solver.robots(), "gaze");
          return std::make_shared<mc_tasks::GazeTask>(robot.frame(config("frame")));
        }
      }();
      t->load(solver, config);
      return t;
    });
} // namespace
