#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

namespace
{

/** Saturate integrator outputs.
 *
 * \param taskName Name of caller AdmittanceTask.
 *
 * \param vector Integrator output vector.
 *
 * \param bound Output (symmetric) bounds.
 *
 * \param label Name of output vector.
 *
 * \param isClamping Map of booleans describing the clamping state for each
 * direction in ['x', 'y', 'z'].
 *
 */
void clampAndWarn(const std::string & taskName, Eigen::Vector3d & vector, const Eigen::Vector3d & bound, const std::string & label, std::map<char, bool> & isClamping)
{
  const char dirName[] = {'x', 'y', 'z'};
  for (unsigned i = 0; i < 3; i++)
  {
    char dir = dirName[i];
    if (vector(i) < -bound(i))
    {
      vector(i) = -bound(i);
      if (!isClamping[dir])
      {
        LOG_WARNING(taskName << ": clamping " << dir << " " << label << " to " << -bound(i));
        isClamping[dir] = true;
      }
    }
    else if (vector(i) > bound(i))
    {
      vector(i) = bound(i);
      if (!isClamping[dir])
      {
        LOG_WARNING(taskName << ": clamping " << dir << " " << label << " to " << bound(i));
        isClamping[dir] = true;
      }
    }
    else if (isClamping[dir])
    {
      LOG_WARNING(taskName << ": " << dir << " " << label << " back within range");
      isClamping[dir] = false;
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
    robot_(robots.robots()[robotIndex]),
    surface_(robots.robot(robotIndex).surface(surfaceName)),
    sensor_(robot_.bodyForceSensor(surface_.bodyName())),
    X_fsactual_surf_(surface_.X_b_s() * sensor_.X_fsactual_parent()),
    timestep_(timestep)
{
  name_ = "admittance_" + robot_.name() + "_" + surfaceName;
}

void AdmittanceTask::update()
{
  wrenchError_ = measuredWrench() - targetWrench_;

  Eigen::Vector3d linearVel = admittance_.force().cwiseProduct(wrenchError_.force());
  Eigen::Vector3d angularVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampAndWarn(name_, linearVel, maxLinearVel_, "linear velocity", isClampingLinearVel_);
  clampAndWarn(name_, angularVel, maxAngularVel_, "angular velocity", isClampingAngularVel_);
  refVelB_ = sva::MotionVecd(angularVel, linearVel);

  // SC: we could do add an anti-windup strategy here, e.g. back-calculation.
  // Yet, keep in mind that our velocity bounds are artificial. Whenever
  // possible, the best is to set to gains so that they are not saturated.

  this->refVelB(refVelB_);
}

void AdmittanceTask::reset()
{
  SurfaceTransformTask::reset();
  admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  refVelB_ = sva::MotionVecd(Eigen::Vector6d::Zero());
  wrenchError_ = sva::ForceVecd(Eigen::Vector6d::Zero());
}

void AdmittanceTask::addToLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::addToLogger(logger);
  logger.removeLogEntry(name_ + "_target_pose");
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
  logger.addLogEntry(name_ + "_ref_vel_body",
                     [this]() -> sva::MotionVecd
                     {
                     return refVelB_;
                     });
  logger.addLogEntry(name_ + "_target_wrench",
                     [this]() -> const sva::ForceVecd &
                     {
                     return targetWrench_;
                     });
  logger.addLogEntry(name_ + "_world_measured_wrench",
                     [this]() -> sva::ForceVecd
                     {
                     return worldMeasuredWrench();
                     });
}

void AdmittanceTask::removeFromLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_admittance");
  logger.removeLogEntry(name_ + "_measured_wrench");
  logger.removeLogEntry(name_ + "_ref_vel_body");
  logger.removeLogEntry(name_ + "_target_wrench");
  logger.removeLogEntry(name_ + "_world_measured_wrench");
}


std::function<bool(const mc_tasks::MetaTask&, std::string&)>
  AdmittanceTask::buildCompletionCriteria(double dt,
                                          const mc_rtc::Configuration & config) const
{
  if(config.has("wrench"))
  {
    sva::ForceVecd target_w = config("wrench");
    Eigen::Vector6d target = target_w.vector();
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    for(size_t i = 0; i < 6; ++i)
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
    return [dof,target](const mc_tasks::MetaTask & t, std::string & out)
    {
      const auto & self = static_cast<const mc_tasks::AdmittanceTask&>(t);
      Eigen::Vector6d w = self.measuredWrench().vector();
      for(size_t i = 0; i < 6; ++i)
      {
        if(dof(i)*fabs(w(i)) < target(i))
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
  gui.addElement(
    mc_rtc::gui::Element<sva::PTransformd>{
      {"Tasks", name_, "pos_target"},
      [this](){ return this->targetPose(); },
      [this](const sva::PTransformd & pos) { this->targetPose(pos); }
    },
    mc_rtc::gui::Transform{}
  );
  gui.addElement(
    mc_rtc::gui::Element<sva::PTransformd>{
      {"Tasks", name_, "pos"},
      std::function<sva::PTransformd()>{
        [this]()
        {
          return robots.robot(rIndex).surface(surfaceName).X_0_s(robots.robot(rIndex));
        }
      }
    },
    mc_rtc::gui::Transform{}
  );
  gui.addElement(
    mc_rtc::gui::Element<Eigen::Vector6d>{
      {"Tasks", name_, "admittance"},
      [this](){ return this->admittance().vector(); },
      [this](const Eigen::Vector6d & a) { this->admittance({a}); }
    },
    mc_rtc::gui::Input<sva::ForceVecd>{ {"cx", "cy", "cz", "fx", "fy" , "fz"} }
  );
  gui.addElement(
    mc_rtc::gui::Element<Eigen::Vector6d>{
      {"Tasks", name_, "wrench"},
      [this](){ return this->targetWrench().vector(); },
      [this](const Eigen::Vector6d & w) { this->targetWrench({w}); }
    },
    mc_rtc::gui::Input<sva::ForceVecd>{ {"cx", "cy", "cz", "fx", "fy" , "fz"} }
  );
  // Don't add SurfaceTransformTask as target configuration is different
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);
}

} // mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("admittance",
  [](mc_solver::QPSolver & solver,
     const mc_rtc::Configuration & config)
  {
    auto t = std::make_shared<mc_tasks::AdmittanceTask>(config("surface"), solver.robots(), config("robotIndex"), solver.dt());
    if(config.has("admittance")) { t->admittance(config("admittance")); }
    if(config.has("damping")) { t->damping(config("damping")); }
    if(config.has("pose")) { t->targetPose(config("pose")); }
    if(config.has("weight")) { t->weight(config("weight")); }
    if(config.has("wrench")) { t->targetWrench(config("wrench")); }
    t->load(solver, config);
    return t;
  }
);

}
