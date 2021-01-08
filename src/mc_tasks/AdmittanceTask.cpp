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

using mc_filter::utils::clampInPlaceAndWarn;

AdmittanceTask::AdmittanceTask(const std::string & surfaceName,
                               const mc_rbdyn::Robots & robots,
                               unsigned int robotIndex,
                               double stiffness,
                               double weight)
: SurfaceTransformTask(surfaceName, robots, robotIndex, stiffness, weight), robots_(robots), rIndex_(robotIndex),
  surface_(robots.robot(robotIndex).surface(surfaceName))
{
  const auto & robot = robots.robot(robotIndex);
  if(!robot.surfaceHasIndirectForceSensor(surfaceName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks::AdmittanceTask] Surface {} does not have a force sensor attached", surfaceName);
  }
  name_ = "admittance_" + robots_.robot(rIndex_).name() + "_" + surfaceName;
  reset();
}

void AdmittanceTask::update(mc_solver::QPSolver &)
{
  // Compute wrench error
  wrenchError_ = measuredWrench() - targetWrench_;

  // Compute linear and angular velocity based on wrench error and admittance
  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());

  // Clamp both values in order to have a 'security'
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");

  // Filter
  refVelB_ = velFilterGain_ * refVelB_ + (1 - velFilterGain_) * sva::MotionVecd(angularVel, linearVel);

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
    mc_rtc::log::warning("[{}] property \"targetPose\" is deprecated, use \"target\" instead", name());
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
  if(config.has("maxVel"))
  {
    sva::MotionVecd maxVel = config("maxVel");
    maxLinearVel(maxVel.linear());
    maxAngularVel(maxVel.angular());
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
  logger.addLogEntry(name_ + "_vel_filter_gain", [this]() { return velFilterGain_; });
}

void AdmittanceTask::removeFromLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_admittance");
  logger.removeLogEntry(name_ + "_measured_wrench");
  logger.removeLogEntry(name_ + "_target_body_vel");
  logger.removeLogEntry(name_ + "_target_wrench");
  logger.removeLogEntry(name_ + "_vel_filter_gain");
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
                                         [this]() { return this->measuredWrench().vector(); }),
                 mc_rtc::gui::NumberInput("Velocity filter gain", [this]() { return velFilterGain_; },
                                          [this](double g) { velFilterGain(g); }));
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
      auto t = std::make_shared<mc_tasks::force::AdmittanceTask>(
          config("surface"), solver.robots(), robotIndexFromConfig(config, solver.robots(), "admittance"));

      t->reset();
      t->load(solver, config);
      return t;
    });
}
