#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

namespace
{
  constexpr double MIN_PRESSURE = 0.5;  // [N]
}

CoPTask::CoPTask(const std::string & surfaceName,
      const mc_rbdyn::Robots & robots,
      unsigned int robotIndex,
      double timestep,
      double stiffness, double weight)
  : AdmittanceTask(surfaceName, robots, robotIndex, timestep, stiffness, weight)
{
  name_ = "cop_" + robot_.name() + "_" + surfaceName;
}

void CoPTask::reset()
{
  targetCoP_ = Eigen::Vector2d::Zero();
  targetForce_ = Eigen::Vector3d::Zero();
  AdmittanceTask::reset();
}

void CoPTask::update()
{
  const double pressure = measuredWrench().force()(2);
  if (pressure < MIN_PRESSURE && (admittance_.couple()(0) > 1e-6 || admittance_.couple()(1) > 1e-6))
  {
    if (!cop_tracking_disabled_)
    {
      LOG_WARNING("Pressure on " << surface_.name() << " < 0, skipping CoP tracking update...");
      cop_tracking_disabled_ = true;
    }
  }
  else
  {
    if (cop_tracking_disabled_)
    {
      LOG_INFO("Pressure is back on " << surface_.name() << ", resuming CoP tracking");
      cop_tracking_disabled_ = false;
    }
    const Eigen::Vector3d targetTorque(+targetCoP_(1) * pressure, -targetCoP_(0) * pressure, 0.);
    AdmittanceTask::targetWrench(sva::ForceVecd(targetTorque, targetForce_));
    AdmittanceTask::update();
  }
}

Eigen::Vector2d CoPTask::measuredCoP() const
{
  const sva::ForceVecd w_surf = measuredWrench();
  const double pressure = w_surf.force()(2);
  if (pressure < MIN_PRESSURE)
  {
    return Eigen::Vector2d::Zero();
  }
  const Eigen::Vector3d tau_surf = w_surf.couple();
  return Eigen::Vector2d(-tau_surf(1) / pressure, +tau_surf(0) / pressure);
}

sva::PTransformd CoPTask::worldMeasuredCoP() const
{
  Eigen::Vector2d cop = measuredCoP();
  sva::PTransformd X_surface_cop {Eigen::Vector3d{cop.x(), cop.y(), 0}};
  sva::PTransformd X_0_surface = robot_.surface(surfaceName).X_0_s(robot_);
  return X_surface_cop * X_0_surface;
}

sva::PTransformd CoPTask::worldTargetCoP() const
{
  const Eigen::Vector2d & cop = targetCoP_;
  sva::PTransformd X_surface_cop {Eigen::Vector3d{cop.x(), cop.y(), 0}};
  sva::PTransformd X_0_surface = surfacePose();
  return X_surface_cop * X_0_surface;
}

void CoPTask::worldTargetCoP(const Eigen::Vector3d & worldCoP)
{
  //
  // For now I cannot tell which of the two following options is the best:
  //
  // (1) sva::PTransformd X_contact_0 = targetPose().inv();
  // (2) sva::PTransformd X_contact_0 = surfacePose().inv();
  //
  // As long as contact is maintained they should perform the same. However,
  // due to the flexibility it may actually be better to use targetPose() (the
  // estimated contact frame, after flexibility) rather than surfacePose (the
  // foot frame below the ankle, before flexibility).
  //
  sva::PTransformd X_contact_0 = surfacePose().inv();
  sva::PTransformd X_0_cop(worldCoP);
  sva::PTransformd X_contact_cop = X_0_cop * X_contact_0;
  Eigen::Vector2d contactCoP(X_contact_cop.translation()(0), X_contact_cop.translation()(1));
  targetCoP(contactCoP);
}

void CoPTask::targetCoP(const Eigen::Vector2d & targetCoP)
{
  targetCoP_ = targetCoP;
}

const Eigen::Vector2d & CoPTask::targetCoP() const
{
  Eigen::Vector2d cop = measuredCoP();
  sva::PTransformd X_surface_cop {Eigen::Vector3d{cop.x(), cop.y(), 0}};
  sva::PTransformd X_0_surface = robot_.surface(surfaceName).X_0_s(robot_);
  return X_surface_cop * X_0_surface;
}

sva::PTransformd CoPTask::worldTargetCoP() const
{
  const Eigen::Vector2d & cop = targetCoP_;
  sva::PTransformd X_surface_cop {Eigen::Vector3d{cop.x(), cop.y(), 0}};
  sva::PTransformd X_0_surface = surfacePose();
  return X_surface_cop * X_0_surface;
}

void CoPTask::worldTargetCoP(const Eigen::Vector3d & worldCoP)
{
  //
  // For now I cannot tell which of the two following options is the best:
  //
  // (1) sva::PTransformd X_contact_0 = targetPose().inv();
  // (2) sva::PTransformd X_contact_0 = surfacePose().inv();
  //
  // As long as contact is maintained they should perform the same. However,
  // due to the flexibility it may actually be better to use targetPose() (the
  // estimated contact frame, after flexibility) rather than surfacePose (the
  // foot frame below the ankle, before flexibility).
  //
  sva::PTransformd X_contact_0 = surfacePose().inv();
  sva::PTransformd X_0_cop(worldCoP);
  sva::PTransformd X_contact_cop = X_0_cop * X_contact_0;
  Eigen::Vector2d contactCoP(X_contact_cop.translation()(0), X_contact_cop.translation()(1));
  targetCoP(contactCoP);
}

void CoPTask::addToLogger(mc_rtc::Logger & logger)
{
  AdmittanceTask::addToLogger(logger);
  logger.addLogEntry(name_ + "_measured_cop",
                     [this]() -> Eigen::Vector2d
                     {
                     return measuredCoP();
                     });
  logger.addLogEntry(name_ + "_target_cop",
                     [this]() -> const Eigen::Vector2d &
                     {
                     return targetCoP_;
                     });
  logger.addLogEntry(name_ + "_world_measured_cop",
                     [this]() -> Eigen::Vector3d
                     {
                     return worldMeasuredCoP().translation();
                     });
  logger.addLogEntry(name_ + "_world_target_cop",
                     [this]() -> Eigen::Vector3d
                     {
                     return worldTargetCoP().translation();
                     });
  logger.addLogEntry(name_ + "_world_measured_cop",
                     [this]() -> Eigen::Vector3d
                     {
                     return worldMeasuredCoP().translation();
                     });
  logger.addLogEntry(name_ + "_world_target_cop",
                     [this]() -> Eigen::Vector3d
                     {
                     return worldTargetCoP().translation();
                     });
}

void CoPTask::removeFromLogger(mc_rtc::Logger & logger)
{
  AdmittanceTask::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_measured_cop");
  logger.removeLogEntry(name_ + "_target_cop");
}

std::function<bool(const mc_tasks::MetaTask&, std::string&)>
  CoPTask::buildCompletionCriteria(double dt,
                                   const mc_rtc::Configuration & config) const
{
  if(config.has("copError"))
  {
    double copError = config("copError");
    assert(copError >= 0);
    return [copError](const mc_tasks::MetaTask & t, std::string & out)
    {
      const auto & self = static_cast<const CoPTask&>(t);
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
    for(size_t i = 0; i < 3; ++i)
    {
      if(std::isnan(force(i))) { dof(i) = 0.; force(i) = 0.; }
      else if(force(i) < 0) { dof(i) = -1.; }
    }
    return [dof,force](const mc_tasks::MetaTask & t, std::string & out)
    {
      const auto & self = static_cast<const CoPTask&>(t);
      Eigen::Vector3d f = self.measuredWrench().force();
      for(size_t i = 0; i < 3; ++i)
      {
        if(dof(i)*fabs(f(i)) < force(i))
        {
          return false;
        }
      }
      out += "force";
      return true;
    };
  }
  return AdmittanceTask::buildCompletionCriteria(dt, config);
}

} // mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("cop",
  [](mc_solver::QPSolver & solver,
     const mc_rtc::Configuration & config)
  {
    auto t = std::make_shared<mc_tasks::CoPTask>(config("surface"), solver.robots(), config("robotIndex"), solver.dt());
    if(config.has("admittance")) { t->admittance(config("admittance")); }
    if(config.has("cop")) { t->targetCoP(config("cop")); }
    if(config.has("damping")) { t->damping(config("damping")); }
    if(config.has("force")) { t->targetForce(config("force")); }
    if(config.has("pose")) { t->targetPose(config("pose")); }
    if(config.has("weight")) { t->weight(config("weight")); }
    t->load(solver, config);
    return t;
  }
);

}
