#include <mc_tasks/AdmittanceTask.h>
#include <mc_rbdyn/rpy_utils.h>

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

AdmittanceTask::AdmittanceTask(const std::string & robotSurface,
      const mc_rbdyn::Robots & robots,
      unsigned int robotIndex,
      double timestep,
      double stiffness, double weight)
  : surface_(robots.robot(robotIndex).surface(robotSurface)),
    wrenchError_(Eigen::Vector6d::Zero()),
    targetWrench_(Eigen::Vector6d::Zero()),
    robot_(robots.robots()[robotIndex]),
    sensor_(robot_.bodyForceSensor(surface_.bodyName())),
    timestep_(timestep),
    admittance_(Eigen::Vector6d::Zero()),
    trans_target_delta_(Eigen::Vector3d::Zero()),
    rpy_target_delta_(Eigen::Vector3d::Zero()),
    X_fsactual_surf_(surface_.X_b_s() * sensor_.X_fsactual_parent())
{
  surfaceTask_ = std::make_shared<SurfaceTransformTask>(robotSurface, robots, robotIndex, stiffness, weight);
  X_0_target_ = surfaceTask_->target();
}

void AdmittanceTask::update()
{
  sva::ForceVecd w_fsactual = sensor_.removeGravity(robot_);
  sva::ForceVecd w_surf = X_fsactual_surf_.dualMul(w_fsactual);

  wrenchError_ = w_surf - targetWrench_; // NB: measured - desired
  Eigen::Vector3d transVel = admittance_.force().cwiseProduct(wrenchError_.force());
  clampAndWarn(transVel, maxTransVel_, "linear velocity");
  trans_target_delta_ += timestep_ * transVel;
  clampAndWarn(trans_target_delta_, maxTransPos_, "linear position");

  Eigen::Vector3d rpyVel = admittance_.couple().cwiseProduct(wrenchError_.couple());
  clampAndWarn(rpyVel, maxRpyVel_, "angular velocity");
  rpy_target_delta_ += timestep_ * rpyVel;
  clampAndWarn(rpy_target_delta_, maxRpyPos_, "angular position");

  const Eigen::Matrix3d R_target_delta = mc_rbdyn::rpyToMat(rpy_target_delta_);
  X_target_delta_ = sva::PTransformd(R_target_delta, trans_target_delta_);
  surfaceTask_->target(X_target_delta_ * X_0_target_);

  /* Does nothing for now, but is here in case of changes */
  MetaTask::update(*surfaceTask_);
}

void AdmittanceTask::addToSolver(mc_solver::QPSolver & solver)
{
  MetaTask::addToSolver(*surfaceTask_, solver);
}

void AdmittanceTask::dimWeight(const Eigen::VectorXd & dimW)
{
  surfaceTask_->dimWeight(dimW);
}

Eigen::VectorXd AdmittanceTask::dimWeight() const
{
  return surfaceTask_->dimWeight();
}

void AdmittanceTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*surfaceTask_, solver);
}

void AdmittanceTask::reset()
{
  surfaceTask_->reset();
  X_0_target_ = surfaceTask_->target();
  targetWrench_ = sva::ForceVecd(Eigen::Vector6d::Zero());
  admittance_ = sva::ForceVecd(Eigen::Vector6d::Zero());
}

void AdmittanceTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  surfaceTask_->resetJointsSelector(solver);
}

void AdmittanceTask::selectActiveJoints(mc_solver::QPSolver & solver,
    const std::vector<std::string> & activeJointsName)
{
  surfaceTask_->selectActiveJoints(solver, activeJointsName);
}

void AdmittanceTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
    const std::vector<std::string> & unactiveJointsName)
{
  surfaceTask_->selectUnactiveJoints(solver, unactiveJointsName);
}

void AdmittanceTask::stiffness(double w)
{
  surfaceTask_->stiffness(w);
}

double AdmittanceTask::stiffness() const
{
  return surfaceTask_->stiffness();
}

void AdmittanceTask::weight(double w)
{
  surfaceTask_->weight(w);
}

double AdmittanceTask::weight() const
{
  return surfaceTask_->weight();
}

} // mc_tasks
