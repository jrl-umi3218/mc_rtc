/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/LookAtTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

LookAtTask::LookAtTask(const std::string & bodyName,
                       const Eigen::Vector3d & bodyVector,
                       const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       double stiffness,
                       double weight)
: LookAtTask(robots.robot(robotIndex).frame(bodyName), bodyVector, stiffness, weight)
{
}

LookAtTask::LookAtTask(const std::string & bodyName,
                       const Eigen::Vector3d & bodyVector,
                       const Eigen::Vector3d & targetPos,
                       const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       double stiffness,
                       double weight)
: LookAtTask(robots.robot(robotIndex).frame(bodyName), bodyVector, stiffness, weight)
{
  target(targetPos);
}

LookAtTask::LookAtTask(const mc_rbdyn::RobotFrame & frame,
                       const Eigen::Vector3d & frameVector,
                       double stiffness,
                       double weight)
: VectorOrientationTask(frame, frameVector, stiffness, weight)
{
  type_ = "lookAt";
  name_ = "look_at_" + frame.robot().name() + "_" + frame.name();
}

void LookAtTask::reset()
{
  VectorOrientationTask::reset();
  target_pos_ = frame_->position().translation() + actual();
}

void LookAtTask::target(const Eigen::Vector3d & pos)
{
  target_pos_ = pos;
  auto target_ori = (pos - frame_->position().translation()).normalized();
  VectorOrientationTask::targetVector(target_ori);
}

Eigen::Vector3d LookAtTask::target() const
{
  return target_pos_;
}
void LookAtTask::addToLogger(mc_rtc::Logger & logger)
{
  VectorOrientationTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_target_pos", this, [this]() -> const Eigen::Vector3d { return target(); });
  logger.addLogEntry(name_ + "_current_pos", this, [this]() { return frame_->position().translation(); });
}

void LookAtTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  VectorOrientationTask::addToGUI(gui);

  gui.addElement({"Tasks", name_}, mc_rtc::gui::Point3D(
                                       "Target Point", [this]() { return this->target(); },
                                       [this](const Eigen::Vector3d & pos) { this->target(pos); }));
}

void LookAtTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  VectorOrientationTask::load(solver, config);
  if(config.has("targetPos"))
  {
    target(config("targetPos"));
  }
}

} // namespace mc_tasks

namespace
{
static auto registered_lookat = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAt",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = [&]() {
        if(config.has("body"))
        {
          return std::make_shared<mc_tasks::LookAtTask>(config("body"), config("bodyVector"), solver.robots(),
                                                        robotIndexFromConfig(config, solver.robots(), "lookAt"));
        }
        else
        {
          const auto & robot = robotFromConfig(config, solver.robots(), "lookAt");
          return std::make_shared<mc_tasks::LookAtTask>(robot.frame(config("frame")), config("frameVector"));
        }
      }();
      t->load(solver, config);
      return t;
    });
}
