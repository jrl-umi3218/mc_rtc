/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/AdmittanceTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Transform.h>

#include <mc_rtc/deprecated.h>

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
: AdmittanceTask(robots.robot(robotIndex).frame(surfaceName), stiffness, weight)
{
}

AdmittanceTask::AdmittanceTask(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: TransformTask(frame, stiffness, weight)
{
  if(!frame.hasForceSensor())
  {
    mc_rtc::log::error_and_throw("[mc_tasks::AdmittanceTask] Frame {} does not have a force sensor attached",
                                 frame.name());
  }
  name_ = "admittance_" + frame.robot().name() + "_" + frame.name();
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

  // Acceleration
  TransformTask::refAccel((refVelB_ + feedforwardVelB_ - TransformTask::refVelB()) / timestep_);

  // Velocity
  TransformTask::refVelB(refVelB_ + feedforwardVelB_);

  // Position
  target(delta * target());
}

void AdmittanceTask::reset()
{
  TransformTask::reset();
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
  TransformTask::load(solver, config);
}

void AdmittanceTask::addToLogger(mc_rtc::Logger & logger)
{
  TransformTask::addToLogger(logger);
  MC_RTC_LOG_HELPER(name_ + "_admittance", admittance_);
  MC_RTC_LOG_HELPER(name_ + "_measured_wrench", measuredWrench);
  MC_RTC_LOG_HELPER(name_ + "_target_body_vel", feedforwardVelB_);
  MC_RTC_LOG_HELPER(name_ + "_target_wrench", targetWrench_);
  MC_RTC_LOG_HELPER(name_ + "_vel_filter_gain", velFilterGain_);
}

void AdmittanceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::Transform(
          "pos_target", [this]() { return this->targetPose(); },
          [this](const sva::PTransformd & pos) { this->targetPose(pos); }),
      mc_rtc::gui::Transform("pos", [this]() { return frame_->position(); }),
      mc_rtc::gui::ArrayInput(
          "admittance", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() { return this->admittance().vector(); },
          [this](const Eigen::Vector6d & a) { this->admittance(a); }),
      mc_rtc::gui::ArrayInput(
          "wrench", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() { return this->targetWrench().vector(); },
          [this](const Eigen::Vector6d & a) { this->targetWrench(a); }),
      mc_rtc::gui::ArrayLabel("measured_wrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                              [this]() { return this->measuredWrench().vector(); }),
      mc_rtc::gui::NumberInput(
          "Velocity filter gain", [this]() { return velFilterGain_; }, [this](double g) { velFilterGain(g); }));
  // Don't add TransformTask as target configuration is different
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);
}

void AdmittanceTask::addToSolver(mc_solver::QPSolver & solver)
{
  timestep_ = solver.dt();
  TransformTask::addToSolver(solver);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "admittance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto frame = [&]() -> std::string {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("AdmittanceTaskLoader", "surface", "frame");
          return config("surface");
        }
        return config("frame");
      }();
      auto rIndex = robotIndexFromConfig(config, solver.robots(), "admittance");
      auto t = std::make_shared<mc_tasks::force::AdmittanceTask>(solver.robots().robot(rIndex).frame(frame));
      t->reset();
      t->load(solver, config);
      return t;
    });
}
