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
: LookAtTask(bodyName, bodyVector, bodyVector, robots, robotIndex, stiffness, weight)
{
  reset();
}

LookAtTask::LookAtTask(const std::string & bodyName,
                       const Eigen::Vector3d & bodyVector,
                       const Eigen::Vector3d & targetPos,
                       const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       double stiffness,
                       double weight)
: VectorOrientationTask(bodyName, bodyVector, robots, robotIndex, stiffness, weight)
{
  const auto & robot = robots.robot(robotIndex);
  bIndex = robot.bodyIndexByName(bodyName);
  type_ = "lookAt";
  name_ = "look_at_" + robot.name() + "_" + bodyName;
  target(targetPos);
}

void LookAtTask::reset()
{
  VectorOrientationTask::reset();
  const auto & robot = robots.robot(rIndex);
  target_pos_ = robot.bodyPosW()[bIndex].translation() + actual();
}

void LookAtTask::target(const Eigen::Vector3d & pos)
{
  target_pos_ = pos;
  const sva::PTransformd & X_0_b = robots.robot(rIndex).mbc().bodyPosW[bIndex];
  auto target_ori = (pos - X_0_b.translation()).normalized();
  VectorOrientationTask::targetVector(target_ori);
}

Eigen::Vector3d LookAtTask::target() const
{
  return target_pos_;
}
void LookAtTask::addToLogger(mc_rtc::Logger & logger)
{
  VectorOrientationTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_target_pos", [this]() -> const Eigen::Vector3d { return target(); });
  logger.addLogEntry(name_ + "_current_pos", [this]() -> const Eigen::Vector3d & {
    return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation();
    ;
  });
}

void LookAtTask::removeFromLogger(mc_rtc::Logger & logger)
{
  VectorOrientationTask::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_target_pos");
  logger.removeLogEntry(name_ + "_current_pos");
}

void LookAtTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  VectorOrientationTask::addToGUI(gui);

  gui.addElement({"Tasks", name_}, mc_rtc::gui::Point3D("Target Point", [this]() { return this->target(); },
                                                        [this](const Eigen::Vector3d & pos) { this->target(pos); }));
}

} // namespace mc_tasks

namespace
{
static auto registered_lookat = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAt",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::LookAtTask>(config("body"), config("bodyVector"), solver.robots(),
                                                      robotIndexFromConfig(config, solver.robots(), "lookAt"));
      if(config.has("weight"))
      {
        t->weight(config("weight"));
      }
      if(config.has("stiffness"))
      {
        auto s = config("stiffness");
        if(s.size())
        {
          Eigen::VectorXd st = s;
          t->stiffness(st);
        }
        else
        {
          t->stiffness(static_cast<double>(s));
        }
      }
      t->load(solver, config);
      if(config.has("targetPos"))
      {
        t->target(config("targetPos"));
      }
      if(config.has("targetVector"))
      {
        t->targetVector(config("targetVector"));
      }
      if(config.has("relativeVector"))
      {
        auto bodyPos = solver.robots().robot(robotNameFromConfig(config, solver.robots(), t->name())).posW();
        bodyPos.translation() = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = config("relativeVector");
        sva::PTransformd target{v};
        t->targetVector((target * bodyPos).translation());
      }
      return t;
    });
}
