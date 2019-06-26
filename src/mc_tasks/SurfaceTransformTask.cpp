/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/SurfaceTransformTask.h>

namespace mc_tasks
{

SurfaceTransformTask::SurfaceTransformTask(const std::string & surfaceName,
                                           const mc_rbdyn::Robots & robots,
                                           unsigned int robotIndex,
                                           double stiffness,
                                           double weight)
: TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>(robots, robotIndex, stiffness, weight),
  surfaceName(surfaceName)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  std::string bodyName = robot.surface(surfaceName).bodyName();
  sva::PTransformd curPos = robot.surface(surfaceName).X_0_s(robot);
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, curPos, robot.surface(surfaceName).X_b_s());

  type_ = "surfaceTransform";
  name_ = "surface_transform_" + robot.name() + "_" + surfaceName;
}

void SurfaceTransformTask::reset()
{
  TrajectoryTaskGeneric::reset();
  const auto & robot = robots.robot(rIndex);
  sva::PTransformd curPos = robot.surface(surfaceName).X_0_s(robot);
  errorT->target(curPos);
}

sva::PTransformd SurfaceTransformTask::target()
{
  return errorT->target();
}

void SurfaceTransformTask::target(const sva::PTransformd & pose)
{
  errorT->target(pose);
}

void SurfaceTransformTask::targetSurface(unsigned int robotIndex,
                                         const std::string & surfaceName,
                                         const sva::PTransformd & offset)
{
  const auto & robot = robots.robot(robotIndex);
  sva::PTransformd targetSurface = robot.surface(surfaceName).X_0_s(robot);
  sva::PTransformd targetPos = offset * targetSurface;
  target(targetPos);
}

void SurfaceTransformTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_surface_pose", [this]() {
    const auto & robot = robots.robot();
    return robot.surface(surfaceName).X_0_s(robot);
  });
  logger.addLogEntry(name_ + "_target_pose", [this]() { return target(); });
}

void SurfaceTransformTask::removeFromLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_surface_pose");
  logger.removeLogEntry(name_ + "_target_pose");
}

void SurfaceTransformTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Transform("pos_target", [this]() { return this->target(); },
                                        [this](const sva::PTransformd & pos) { this->target(pos); }),
                 mc_rtc::gui::Transform("pos", [this]() {
                   return robots.robot(rIndex).surface(surfaceName).X_0_s(robots.robot(rIndex));
                 }));
}

} // namespace mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "surfaceTransform",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t =
          std::make_shared<mc_tasks::SurfaceTransformTask>(config("surface"), solver.robots(), config("robotIndex"));

      if(config.has("targetSurface"))
      {
        const auto & c = config("targetSurface");
        t->targetSurface(c("robotIndex"), c("surface"),
                         {c("offset_rotation", Eigen::Matrix3d::Identity().eval()),
                          c("offset_translation", Eigen::Vector3d::Zero().eval())});
      }
      else if(config.has("target"))
      {
        t->target(config("target"));
      }
      else if(config.has("move"))
      {
        sva::PTransformd move = config("move");
        t->target(move * t->target());
      }
      else if(config.has("moveWorld"))
      {
        sva::PTransformd move = config("moveWorld");
        t->target(t->target() * move);
      }
      else if(config.has("relative"))
      {
        const auto & robot = solver.robot(config("robotIndex"));
        std::string s1 = config("relative")("s1");
        std::string s2 = config("relative")("s2");
        sva::PTransformd target = config("relative")("target");
        auto X_0_s1 = robot.surfacePose(s1);
        auto X_0_s2 = robot.surfacePose(s2);
        auto X_s1_s2 = X_0_s2 * X_0_s1.inv();
        X_s1_s2.translation() = X_s1_s2.translation() / 2;
        auto X_0_relative = X_s1_s2 * X_0_s1;
        t->target(target * X_0_relative);
      }
      t->load(solver, config);
      return t;
    });
}
