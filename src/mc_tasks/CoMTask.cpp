/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

static inline tasks::qp::CoMTask * tasks_error(mc_rtc::void_ptr & ptr)
{
  return static_cast<tasks::qp::CoMTask *>(ptr.get());
}

static inline const tasks::qp::CoMTask * tasks_error(const mc_rtc::void_ptr & ptr)
{
  return static_cast<const tasks::qp::CoMTask *>(ptr.get());
}

CoMTask::CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric(robots, robotIndex, stiffness, weight), robot_index_(robotIndex),
  cur_com_(Eigen::Vector3d::Zero())
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);

  cur_com_ = rbd::computeCoM(robot.mb(), robot.mbc());

  switch(backend_)
  {
    case Backend::Tasks:
      finalize<tasks::qp::CoMTask>(robots.mbs(), static_cast<int>(robotIndex), cur_com_);
      break;
    default:
      mc_rtc::log::error_and_throw("[CoMTask] Not implemented for solver backend: {}", backend_);
  }
  type_ = "com";
  name_ = "com_" + robots.robot(robot_index_).name();
}

void CoMTask::reset()
{
  TrajectoryTaskGeneric::reset();
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  cur_com_ = rbd::computeCoM(robot.mb(), robot.mbc());
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->com(cur_com_);
      break;
    default:
      break;
  }
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
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->com(cur_com_);
      break;
    default:
      break;
  }
}

void CoMTask::com(const Eigen::Vector3d & com)
{
  cur_com_ = com;
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->com(cur_com_);
      break;
    default:
      break;
  }
}

const Eigen::Vector3d & CoMTask::com() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(errorT)->com();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

const Eigen::Vector3d & CoMTask::actual() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(errorT)->actual();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void CoMTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  MC_RTC_LOG_HELPER(name_ + "_pos", actual);
  MC_RTC_LOG_HELPER(name_ + "_target", cur_com_);
}

void CoMTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Point3D(
                     "com_target", [this]() -> const Eigen::Vector3d & { return this->com(); },
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
