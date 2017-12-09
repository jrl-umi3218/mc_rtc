#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

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
  AdmittanceTask::reset();
}

void CoPTask::update()
{
  const double pressure = measuredWrench().force()(2);
  if (pressure < MIN_PRESSURE && (admittance_.couple()(0) > 1e-6 || admittance_.couple()(1) > 1e-6))
  {
    LOG_WARNING("Pressure on " << surface_.name() << " < " << MIN_PRESSURE << " [N], "
        << "disabling CoP tracking");
    admittance_.couple()(0) = 0.;
    admittance_.couple()(1) = 0.;
  }
  const Eigen::Vector3d targetTorque(+targetCoP_(1) * pressure, -targetCoP_(0) * pressure, 0.);
  this->targetWrench(sva::ForceVecd(targetTorque, targetForce_));
  AdmittanceTask::update();
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

void CoPTask::targetCoP(const Eigen::Vector2d & targetCoP)
{
  targetCoP_ = targetCoP;
}

const Eigen::Vector2d & CoPTask::targetCoP() const
{
  return targetCoP_;
}

const Eigen::Vector3d & CoPTask::targetForce() const
{
  return targetForce_;
}

void CoPTask::targetForce(const Eigen::Vector3d & targetForce)
{
  targetForce_ = targetForce;
}

void CoPTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_admittance",
                     [this]() -> const sva::ForceVecd &
                     {
                     return admittance();
                     });
  logger.addLogEntry(name_ + "_measured_cop",
                     [this]() -> Eigen::Vector2d
                     {
                     return measuredCoP();
                     });
  logger.addLogEntry(name_ + "_measured_force",
                     [this]() -> Eigen::Vector3d
                     {
                     return measuredWrench().force();
                     });
  logger.addLogEntry(name_ + "_target_cop",
                     [this]() -> const Eigen::Vector2d &
                     {
                     return targetCoP_;
                     });
  logger.addLogEntry(name_ + "_target_force",
                     [this]() -> const Eigen::Vector3d &
                     {
                     return targetForce_;
                     });
  logger.addLogEntry(name_ + "_target_pose",
                     [this]() -> const sva::PTransformd &
                     {
                     return targetPose();
                     });
}

void CoPTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_admittance");
  logger.removeLogEntry(name_ + "_measured_cop");
  logger.removeLogEntry(name_ + "_measured_force");
  logger.removeLogEntry(name_ + "_target_cop");
  logger.removeLogEntry(name_ + "_target_force");
  logger.removeLogEntry(name_ + "_target_pose");
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
    if(config.has("force")) { t->targetForce(config("force")); }
    if(config.has("pose")) { t->targetPose(config("pose")); }
    if(config.has("stiffness")) { t->stiffness(config("stiffness")); }
    if(config.has("weight")) { t->weight(config("weight")); }
    t->load(solver, config);
    return t;
  }
);

}
