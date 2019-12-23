/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/AdmittanceTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

namespace force
{

AdmittanceTask::AdmittanceTask(const std::string & surfaceName,
                               const mc_rbdyn::Robots & robots,
                               unsigned int robotIndex,
                               double stiffness,
                               double weight)
: SurfaceTransformTask(surfaceName, robots, robotIndex, stiffness, weight), robots_(robots), rIndex_(robotIndex),
  surface_(robots.robot(robotIndex).surface(surfaceName))
{
  name_ = "admittance_" + robots_.robot(rIndex_).name() + "_" + surfaceName;
  reset();
}

void AdmittanceTask::update()
{
  // Compute wrench error
  wrenchError_ = measuredWrench() - targetWrench_;

  // Compute linear and angular velocity based on wrench error and admittance
  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());

  // Clamp both values in order to have a 'security'
  clampAndWarn(name_, linearVel, maxLinearVel_, "linear velocity", isClampingLinearVel_);
  clampAndWarn(name_, angularVel, maxAngularVel_, "angular velocity", isClampingAngularVel_);

  // Filter
  refVelB_ = 0.8 * refVelB_ + 0.2 * sva::MotionVecd(angularVel, linearVel);

  // Compute position and rotation delta
  sva::PTransformd delta(mc_rbdyn::rpyToMat(timestep_ * refVelB_.angular()), timestep_ * refVelB_.linear());

  // Apply feed forward term
  refVelB_ += feedforwardVelB_;

  // Acceleration
  SurfaceTransformTask::refAccel((refVelB_ - SurfaceTransformTask::refVelB()) / timestep_);

  // Velocity
  SurfaceTransformTask::refVelB(refVelB_);

  // Position
  target(delta * target());
}

void AdmittanceTask::reset()
{
  SurfaceTransformTask::reset();
  admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  feedforwardVelB_ = sva::MotionVecd(Eigen::Vector6d::Zero());
  targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());
}

/*! \brief Load parameters from a Configuration object */
void AdmittanceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("admittance"))
  {
    admittance(config("admittance"));
  }
  else if(config.has("targetPose"))
  {
    targetPose(config("targetPose"));
  }
  if(config.has("wrench"))
  {
    targetWrench(config("wrench"));
  }
  if(config.has("refVelB"))
  {
    refVelB(config("refVelB"));
  }
  SurfaceTransformTask::load(solver, config);
}

void AdmittanceTask::addToLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_admittance", [this]() -> const sva::ForceVecd & { return admittance_; });
  logger.addLogEntry(name_ + "_measured_wrench", [this]() -> sva::ForceVecd { return measuredWrench(); });
  logger.addLogEntry(name_ + "_target_body_vel", [this]() -> const sva::MotionVecd & { return feedforwardVelB_; });
  logger.addLogEntry(name_ + "_target_wrench", [this]() -> const sva::ForceVecd & { return targetWrench_; });
}

void AdmittanceTask::removeFromLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_admittance");
  logger.removeLogEntry(name_ + "_measured_wrench");
  logger.removeLogEntry(name_ + "_target_body_vel");
  logger.removeLogEntry(name_ + "_target_wrench");
}

std::function<bool(const mc_tasks::MetaTask &, std::string &)> AdmittanceTask::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{
  if(config.has("wrench"))
  {
    sva::ForceVecd target_w = config("wrench");
    Eigen::Vector6d target = target_w.vector();
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    for(int i = 0; i < 6; ++i)
    {
      if(std::isnan(target(i)))
      {
        dof(i) = 0.;
        target(i) = 0.;
      }
      else if(target(i) < 0)
      {
        dof(i) = -1.;
      }
    }
    return [dof, target](const mc_tasks::MetaTask & t, std::string & out) {
      const auto & self = static_cast<const mc_tasks::force::AdmittanceTask &>(t);
      Eigen::Vector6d w = self.measuredWrench().vector();
      for(int i = 0; i < 6; ++i)
      {
        if(dof(i) * fabs(w(i)) < target(i))
        {
          return false;
        }
      }
      out += "wrench";
      return true;
    };
  }
  return MetaTask::buildCompletionCriteria(dt, config);
}

void AdmittanceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Transform("pos_target", [this]() { return this->targetPose(); },
                                        [this](const sva::PTransformd & pos) { this->targetPose(pos); }),
                 mc_rtc::gui::Transform(
                     "pos", [this]() { return robots.robot(rIndex).surface(surfaceName).X_0_s(robots.robot(rIndex)); }),
                 mc_rtc::gui::ArrayInput("admittance", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->admittance().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->admittance(a); }),
                 mc_rtc::gui::ArrayInput("wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->targetWrench().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->targetWrench(a); }),
                 mc_rtc::gui::ArrayLabel("measured_wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->measuredWrench().vector(); }));
  // Don't add SurfaceTransformTask as target configuration is different
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);
}

void AdmittanceTask::addToSolver(mc_solver::QPSolver & solver)
{
  timestep_ = solver.dt();
  SurfaceTransformTask::addToSolver(solver);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "admittance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t =
          std::make_shared<mc_tasks::force::AdmittanceTask>(config("surface"), solver.robots(), config("robotIndex"));
      t->load(solver, config);
      return t;
    });
}
