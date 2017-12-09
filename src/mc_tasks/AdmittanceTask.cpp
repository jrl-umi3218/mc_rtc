#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

namespace
{

void clampAndWarn(Eigen::Vector3d & vector, const Eigen::Vector3d & bound, const std::string & label)
{
  const char dirName[] = {'x', 'y', 'z'};
  for (unsigned i = 0; i < 3; i++)
  {
    if (vector(i) < -bound(i))
    {
      LOG_WARNING("AdmittanceTask: " << label << " hit lower bound along " << dirName[i] << "-coordinate");
      vector(i) = -bound(i);
    }
    else if (vector(i) > bound(i))
    {
      LOG_WARNING("AdmittanceTask: " << label << " hit upper bound along " << dirName[i] << "-coordinate");
      vector(i) = bound(i);
    }
  }
}

}

AdmittanceTask::AdmittanceTask(const std::string & surfaceName,
      const mc_rbdyn::Robots & robots,
      unsigned int robotIndex,
      double timestep,
      double stiffness, double weight)
  : SurfaceTransformTask(surfaceName, robots, robotIndex, stiffness, weight), 
    surface_(robots.robot(robotIndex).surface(surfaceName)),
    robot_(robots.robots()[robotIndex]),
    sensor_(robot_.bodyForceSensor(surface_.bodyName())),
    timestep_(timestep),
    X_fsactual_surf_(surface_.X_b_s() * sensor_.X_fsactual_parent())
{
  X_0_target_ = SurfaceTransformTask::target();
  name_ = "admittance_" + robot_.name() + "_" + surfaceName;
}

void AdmittanceTask::update()
{
  wrenchError_ = measuredWrench() - targetWrench_;
  Eigen::Vector3d transVel = admittance_.force().cwiseProduct(wrenchError_.force());
  clampAndWarn(transVel, maxTransVel_, "linear velocity");
  trans_target_delta_ += timestep_ * transVel;
  clampAndWarn(trans_target_delta_, maxTransPos_, "linear position");

  Eigen::Vector3d rpyVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampAndWarn(rpyVel, maxRpyVel_, "angular velocity");
  rpy_target_delta_ += timestep_ * rpyVel;
  clampAndWarn(rpy_target_delta_, maxRpyPos_, "angular position");

  const Eigen::Matrix3d R_target_delta = mc_rbdyn::rpyToMat(rpy_target_delta_);
  const sva::PTransformd X_target_delta = sva::PTransformd(R_target_delta, trans_target_delta_);
  this->target(X_target_delta * X_0_target_);
}

void AdmittanceTask::reset()
{
  SurfaceTransformTask::reset();
  X_0_target_ = SurfaceTransformTask::target();
  targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
}

void AdmittanceTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_admittance",
                     [this]() -> const sva::ForceVecd &
                     {
                     return admittance_;
                     });
  logger.addLogEntry(name_ + "_measured_wrench",
                     [this]() -> sva::ForceVecd
                     {
                     return measuredWrench();
                     });
  logger.addLogEntry(name_ + "_target_pose",
                     [this]() -> const sva::PTransformd &
                     {
                     return X_0_target_;
                     });
  logger.addLogEntry(name_ + "_target_wrench",
                     [this]() -> const sva::ForceVecd &
                     {
                     return targetWrench_;
                     });
}

void AdmittanceTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_admittance");
  logger.removeLogEntry(name_ + "_measured_wrench");
  logger.removeLogEntry(name_ + "_target_pose");
  logger.removeLogEntry(name_ + "_target_wrench");
}

} // mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("admittance",
  [](mc_solver::QPSolver & solver,
     const mc_rtc::Configuration & config)
  {
    auto t = std::make_shared<mc_tasks::AdmittanceTask>(config("surface"), solver.robots(), config("robotIndex"), solver.dt());
    if(config.has("stiffness")) { t->stiffness(config("stiffness")); }
    if(config.has("weight")) { t->weight(config("weight")); }
    if(config.has("admittance")) { t->admittance(config("admittance")); }
    if(config.has("pose")) { t->targetPose(config("pose")); }
    if(config.has("wrench")) { t->targetWrench(config("wrench")); }
    t->load(solver, config);
    return t;
  }
);

}
