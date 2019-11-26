/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/CoPTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

#include <mc_rtc/gui/ArrayLabel.h>

namespace mc_tasks
{

namespace force
{

CoPTask::CoPTask(const std::string & surfaceName,
                 const mc_rbdyn::Robots & robots,
                 unsigned int robotIndex,
                 double stiffness,
                 double weight)
: DampingTask(surfaceName, robots, robotIndex, stiffness, weight)
{
  name_ = "cop_" + robots_.robot(rIndex_).name() + "_" + surfaceName;
}

void CoPTask::reset()
{
  targetCoP_ = Eigen::Vector2d::Zero();
  targetForce_ = Eigen::Vector3d::Zero();
  DampingTask::reset();
}

void CoPTask::update()
{
  double pressure = std::max(0., measuredWrench().force().z());
  Eigen::Vector3d targetTorque = {+targetCoP_.y() * pressure, -targetCoP_.x() * pressure, 0.};
  DampingTask::targetWrench({targetTorque, targetForce_});
  DampingTask::update();
}

void CoPTask::addToLogger(mc_rtc::Logger & logger)
{
  DampingTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_measured_cop", [this]() -> Eigen::Vector2d { return measuredCoP(); });
  logger.addLogEntry(name_ + "_target_cop", [this]() -> const Eigen::Vector2d & { return targetCoP_; });
}

void CoPTask::removeFromLogger(mc_rtc::Logger & logger)
{
  DampingTask::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_measured_cop");
  logger.removeLogEntry(name_ + "_measured_copW");
  logger.removeLogEntry(name_ + "_target_cop");
  logger.removeLogEntry(name_ + "_target_copW");
}

void CoPTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::ArrayLabel("cop_measured", [this]() -> Eigen::Vector2d { return this->measuredCoP(); }),
                 mc_rtc::gui::ArrayInput("cop_target",
                                         [this]() -> const Eigen::Vector2d & { return this->targetCoP(); },
                                         [this](const Eigen::Vector2d & cop) { this->targetCoP(cop); }));
  // Don't add SurfaceTransformTask as target configuration is different
  DampingTask::addToGUI(gui);
}

std::function<bool(const mc_tasks::MetaTask &, std::string &)> CoPTask::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{
  if(config.has("copError"))
  {
    double copError = config("copError");
    assert(copError >= 0);
    return [copError](const mc_tasks::MetaTask & t, std::string & out) {
      const auto & self = static_cast<const CoPTask &>(t);
      Eigen::Vector2d error = self.measuredCoP() - self.targetCoP();
      if(error.norm() < copError)
      {
        out += "CoP error";
        return true;
      }
      return false;
    };
  }
  if(config.has("force"))
  {
    Eigen::Vector3d force = config("force");
    Eigen::Vector3d dof = Eigen::Vector3d::Ones();
    for(int i = 0; i < 3; ++i)
    {
      if(std::isnan(force(i)))
      {
        dof(i) = 0.;
        force(i) = 0.;
      }
      else if(force(i) < 0)
      {
        dof(i) = -1.;
      }
    }
    return [dof, force](const mc_tasks::MetaTask & t, std::string & out) {
      const auto & self = static_cast<const CoPTask &>(t);
      Eigen::Vector3d f = self.measuredWrench().force();
      for(int i = 0; i < 3; ++i)
      {
        if(dof(i) * fabs(f(i)) < force(i))
        {
          return false;
        }
      }
      out += "force";
      return true;
    };
  }
  return DampingTask::buildCompletionCriteria(dt, config);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "cop",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::force::CoPTask>(config("surface"), solver.robots(), config("robotIndex"));
      if(config.has("admittance"))
      {
        t->admittance(config("admittance"));
      }
      if(config.has("cop"))
      {
        t->targetCoP(config("cop"));
      }
      if(config.has("force"))
      {
        t->targetForce(config("force"));
      }
      if(config.has("targetSurface"))
      {
        const auto & c = config("targetSurface");
        t->targetSurface(c("robotIndex"), c("surface"),
                         {c("offset_rotation", Eigen::Matrix3d::Identity().eval()),
                          c("offset_translation", Eigen::Vector3d::Zero().eval())});
      }
      else if(config.has("targetPose"))
      {
        t->targetPose(config("targetPose"));
      }
      if(config.has("weight"))
      {
        t->weight(config("weight"));
      }
      t->load(solver, config);
      return t;
    });
}
