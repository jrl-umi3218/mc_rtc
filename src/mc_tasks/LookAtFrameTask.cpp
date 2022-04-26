/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/LookAtSurfaceTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

LookAtFrameTask::LookAtFrameTask(const mc_rbdyn::Robots & robots,
                                 unsigned int robotIndex,
                                 const std::string & bodyName,
                                 const Eigen::Vector3d & bodyVector,
                                 unsigned int surfaceRobotIndex,
                                 const std::string & surfaceName,
                                 double stiffness,
                                 double weight)
: LookAtFrameTask(robots.robot(robotIndex).frame(bodyName),
                  bodyVector,
                  robots.robot(surfaceRobotIndex).frame(surfaceName),
                  stiffness,
                  weight)
{
}

LookAtFrameTask::LookAtFrameTask(const mc_rbdyn::RobotFrame & frame,
                                 const Eigen::Vector3d & frameVector,
                                 const mc_rbdyn::Frame & target,
                                 double stiffness,
                                 double weight)
: LookAtTask(frame, frameVector, stiffness, weight), target_(target)
{
  type_ = "lookAtSurface";
  name_ = "look_at_surface_" + frame.robot().name() + "_" + frame.name() + "_" + target.name();
}

void LookAtFrameTask::update(mc_solver::QPSolver &)
{
  LookAtTask::target((offset_ * target_->position()).translation());
}

void LookAtFrameTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  LookAtTask::load(solver, config);
  if(config.has("offset"))
  {
    offset_ = config("offset");
  }
}

} // namespace mc_tasks

namespace
{

std::shared_ptr<mc_tasks::LookAtFrameTask> load_look_at_frame(mc_solver::QPSolver & solver,
                                                              const mc_rtc::Configuration & config)
{
  Eigen::Vector3d frameVector = Eigen::Vector3d::Zero();
  const auto & robot = robotFromConfig(config, solver.robots(), "lookAtFrame");
  const auto & frame = [&]() -> const mc_rbdyn::RobotFrame & {
    if(config.has("body"))
    {
      mc_rtc::log::deprecated("LookAtFrameTaskLoader", "body", "frame");
      frameVector = config("bodyVector");
      return robot.frame(config("body"));
    }
    frameVector = config("frameVector");
    return robot.frame(config("frame"));
  }();
  const auto & target = [&]() -> const mc_rbdyn::RobotFrame & {
    if(config.has("surface"))
    {
      mc_rtc::log::deprecated("LookAtFrameTaskLoader", "surface", "target");
      const auto & target_robots =
          robotFromConfig(config, solver.robots(), "lookAtFrame", false, "surfaceRobotIndex", "surfaceRobot");
      return target_robots.frame(config("surface"));
    }
    auto tgt = config("target");
    const auto & target_robots =
        robotFromConfig(tgt, solver.robots(), "lookAtFrame", false, "robotIndex", "robot", frame.robot().name());
    return target_robots.frame(tgt("frame"));
  }();
  auto t = std::make_shared<mc_tasks::LookAtFrameTask>(frame, frameVector, target);
  t->load(solver, config);
  return t;
}

static auto registered_lookat_surface =
    mc_tasks::MetaTaskLoader::register_load_function("lookAtSurface", &load_look_at_frame);
static auto registered_lookat_frame =
    mc_tasks::MetaTaskLoader::register_load_function("lookAtFrame", &load_look_at_frame);
} // namespace
