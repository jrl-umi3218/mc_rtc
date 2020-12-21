/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

CoMTask::CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::CoMTask>(robots, robotIndex, stiffness, weight), robot_index_(robotIndex),
  cur_com_(Eigen::Vector3d::Zero())
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);

  cur_com_ = rbd::computeCoM(robot.mb(), robot.mbc());

  finalize(robots.mbs(), static_cast<int>(robotIndex), cur_com_);
  type_ = "com";
  name_ = "com_" + robots.robot(robot_index_).name();
}

void CoMTask::reset()
{
  TrajectoryTaskGeneric::reset();
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  cur_com_ = rbd::computeCoM(robot.mb(), robot.mbc());
  errorT->com(cur_com_);
}

void CoMTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TrajectoryBase::load(solver, config);
  if(config.has("com"))
  {
    this->com(config("com"));
  }
  if(config.has("above"))
  {
    auto surfaces = mc_rtc::fromVectorOrElement<std::string>(config, "above");
    auto com = this->com();
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    auto & robot = robotFromConfig(config, solver.robots(), name());
    for(const auto & s : surfaces)
    {
      target += robot.surface(s).X_0_s(robot).translation();
    }
    target /= static_cast<double>(surfaces.size());
    this->com({target.x(), target.y(), com.z()});
  }
  if(config.has("move_com"))
  {
    this->move_com(config("move_com"));
  }
  if(config.has("offset"))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED][" + name()
                         + "] The \"offset\" property is deprecated, use move_com instead");
    Eigen::Vector3d offset = config("offset", Eigen::Vector3d::Zero().eval());
    this->com(this->com() + offset);
  }
}

void CoMTask::move_com(const Eigen::Vector3d & com)
{
  cur_com_ += com;
  errorT->com(cur_com_);
}

void CoMTask::com(const Eigen::Vector3d & com)
{
  cur_com_ = com;
  errorT->com(com);
}

const Eigen::Vector3d & CoMTask::com() const
{
  return errorT->com();
}

const Eigen::Vector3d & CoMTask::actual() const
{
  return errorT->actual();
}

void CoMTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_pos", [this]() -> const Eigen::Vector3d & { return actual(); });
  logger.addLogEntry(name_ + "_target", [this]() -> const Eigen::Vector3d & { return cur_com_; });
}

void CoMTask::removeFromLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_pos");
  logger.removeLogEntry(name_ + "_target");
}

void CoMTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<tasks::qp::CoMTask>::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Point3D("com_target", [this]() -> const Eigen::Vector3d & { return this->com(); },
                                      [this](const Eigen::Vector3d & com) { this->com(com); }),
                 mc_rtc::gui::Point3D("com", [this]() -> const Eigen::Vector3d & { return this->actual(); }));
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "com",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto robotIndex = robotIndexFromConfig(config, solver.robots(), "CoMTask");
      auto t = std::make_shared<mc_tasks::CoMTask>(solver.robots(), robotIndex);
      t->load(solver, config);
      return t;
    });
}
