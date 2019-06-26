/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/DampingTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

namespace force
{

DampingTask::DampingTask(const std::string & surfaceName,
                         const mc_rbdyn::Robots & robots,
                         unsigned int robotIndex,
                         double stiffness,
                         double weight)
: AdmittanceTask(surfaceName, robots, robotIndex, stiffness, weight)
{
  name_ = "damping_" + robots_.robot(robotIndex).name() + "_" + surfaceName;
  reset();
}

void DampingTask::update()
{
  wrenchError_ = measuredWrench() - targetWrench_;

  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampAndWarn(name_, linearVel, maxLinearVel_, "linear velocity", isClampingLinearVel_);
  clampAndWarn(name_, angularVel, maxAngularVel_, "angular velocity", isClampingAngularVel_);
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

static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "damping",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::force::DampingTask>(config("surface"), solver.robots(), config("robotIndex"));
      if(config.has("admittance"))
      {
        t->admittance(config("admittance"));
      }
      if(config.has("damping"))
      {
        double d = config("damping");
        t->damping(d);
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
      if(config.has("wrench"))
      {
        t->targetWrench(config("wrench"));
      }
      t->load(solver, config);
      return t;
    });
}
