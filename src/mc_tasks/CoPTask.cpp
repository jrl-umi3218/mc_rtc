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
}

void CoPTask::update()
{
  const double Fz = measuredWrench().force()(2);
  const Eigen::Vector3d targetTorque(+targetCoP_(1) * Fz, -targetCoP_(0) * Fz, 0.);
  this->targetWrench(sva::ForceVecd(targetTorque, targetForce_));
  AdmittanceTask::update();
}

void CoPTask::reset()
{
  targetCoP_ = Eigen::Vector2d::Zero();
  AdmittanceTask::reset();
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
