/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>
#include <mc_tasks/GazeTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

GazeTask::GazeTask(const std::string & bodyName,
                   const Eigen::Vector2d & point2d,
                   double depthEstimate,
                   const sva::PTransformd & X_b_gaze,
                   const mc_rbdyn::Robots & robots,
                   unsigned int robotIndex,
                   double stiffness,
                   double weight)
: TrajectoryTaskGeneric<tasks::qp::GazeTask>(robots, robotIndex, stiffness, weight)
{
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, point2d, depthEstimate, X_b_gaze);
  type_ = "gaze";
  name_ = "gaze_" + robots.robot(robotIndex).name() + "_" + bodyName;
}

GazeTask::GazeTask(const std::string & bodyName,
                   const Eigen::Vector3d & point3d,
                   const sva::PTransformd & X_b_gaze,
                   const mc_rbdyn::Robots & robots,
                   unsigned int robotIndex,
                   double stiffness,
                   double weight)
: TrajectoryTaskGeneric<tasks::qp::GazeTask>(robots, robotIndex, stiffness, weight)
{
  if(point3d.z() <= 0)
  {
    LOG_ERROR_AND_THROW(std::logic_error, "GazeTask expects the depth estimate to be >0, provided " << point3d.z());
  }
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, point3d, X_b_gaze);
  type_ = "gaze";
  name_ = "gaze_" + robots.robot(robotIndex).name() + "_" + bodyName;
}

void GazeTask::reset()
{
  TrajectoryTaskGeneric::reset();
  errorT->error(Eigen::Vector2d::Zero().eval(), Eigen::Vector2d::Zero());
}

void GazeTask::error(const Eigen::Vector2d & point2d, const Eigen::Vector2d & point2d_ref)
{
  errorT->error(point2d, point2d_ref);
}

void GazeTask::error(const Eigen::Vector3d & point3d, const Eigen::Vector2d & point2d_ref)
{
  if(point3d.z() > 0)
  {
    errorT->error(point3d, point2d_ref);
  }
  else
  {
    LOG_WARNING("GazeTask expects the depth estimate to be >0, provided "
                << point3d.z() << ": ignoring error update for this iteration");
  }
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "gaze",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::GazeTask>(config("body"), Eigen::Vector3d{0, 0, 1}, config("X_b_gaze"),
                                                    solver.robots(), config("robotIndex"));
      t->load(solver, config);
      return t;
    });
}
