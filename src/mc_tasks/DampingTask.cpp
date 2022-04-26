/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/DampingTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_filter/utils/clamp.h>

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

namespace force
{

using mc_filter::utils::clampInPlaceAndWarn;

DampingTask::DampingTask(const std::string & surfaceName,
                         const mc_rbdyn::Robots & robots,
                         unsigned int robotIndex,
                         double stiffness,
                         double weight)
: DampingTask(robots.robot(robotIndex).frame(surfaceName), stiffness, weight)
{
}

DampingTask::DampingTask(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: AdmittanceTask(frame, stiffness, weight)
{
  name_ = "damping_" + frame_->robot().name() + "_" + frame_->name();
  reset();
}

void DampingTask::update(mc_solver::QPSolver &)
{
  wrenchError_ = measuredWrench() - targetWrench_;

  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampInPlaceAndWarn(linearVel, (-maxLinearVel_).eval(), maxLinearVel_, name_ + " linear velocity");
  clampInPlaceAndWarn(angularVel, (-maxAngularVel_).eval(), maxAngularVel_, name_ + " angular velocity");
  refVelB_ = feedforwardVelB_ + sva::MotionVecd{angularVel, linearVel};

  // SC: we could do add an anti-windup strategy here, e.g. back-calculation.
  // Yet, keep in mind that our velocity bounds are artificial. Whenever
  // possible, the best is to set to gains so that they are not saturated.

  SurfaceTransformTask::refVelB(refVelB_);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "damping",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto frame = [&]() -> std::string {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("DampingTaskLoader", "surface", "frame");
          return config("surface");
        }
        return config("frame");
      }();
      auto rIndex = robotIndexFromConfig(config, solver.robots(), "damping");
      auto t = std::make_shared<mc_tasks::force::DampingTask>(solver.robots().robot(rIndex).frame(frame));
      t->reset();
      t->load(solver, config);
      return t;
    });
}
