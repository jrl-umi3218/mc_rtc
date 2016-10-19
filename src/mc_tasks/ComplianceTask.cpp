#include <mc_tasks/ComplianceTask.h>
#include <mc_rbdyn/rpy_utils.h>

namespace mc_tasks
{

namespace
{

std::function<double(double)> clamper(double value)
{
  auto clamp = [value](const double & v) { return std::min(std::max(v, -value), value); };
  return clamp;
}

}

ComplianceTask::ComplianceTask(const mc_rbdyn::Robots & robots,
      unsigned int robotIndex,
      const mc_rbdyn::ForceSensor& forceSensor,
      const std::map<std::string, sva::ForceVecd>& ctlWrenches,
      const mc_rbdyn::ForceSensorsCalibrator& calibrator,
      double timestep,
      const Eigen::Matrix6d& dof,
      double stiffness, double weight, double forceThresh, double torqueThresh,
      std::pair<double, double> forceGain, std::pair<double, double> torqueGain)
  : ctlWrenches_(ctlWrenches),
    wrench_(Eigen::Vector6d::Zero()),
    obj_(Eigen::Vector6d::Zero()),
    error_(Eigen::Vector6d::Zero()),
    errorD_(Eigen::Vector6d::Zero()),
    calibrator_(calibrator),
    robot_(robots.robots()[robotIndex]),
    sensor_(forceSensor),
    timestep_(timestep),
    forceThresh_(forceThresh),
    torqueThresh_(torqueThresh),
    forceGain_(forceGain),
    torqueGain_(torqueGain),
    dof_(dof)
{
  /* FIXME : What we really want is to have the efTask free in the body referential
   * but the current implementation uses the world referential: it works because
   * feet are placed horizontally on the ground */
  efTask_ = std::make_shared<EndEffectorTask>(forceSensor.parentBodyName, robots,
                                              robotIndex, stiffness, weight);
  efTask_->orientationTask->dimWeight(dof_.diagonal().head(3));
  efTask_->positionTask->dimWeight(dof_.diagonal().tail(3));
  clampTrans_ = clamper(0.01);
  clampRot_ = clamper(0.1);
}

ComplianceTask::ComplianceTask(const mc_rbdyn::Robots & robots,
      unsigned int robotIndex,
      const mc_rbdyn::ForceSensor& forceSensor,
      const std::map<std::string, sva::ForceVecd>& ctlWrenches,
      const mc_rbdyn::ForceSensorsCalibrator& calibrator,
      double timestep,
      double stiffness, double weight,
      double forceThresh, double torqueThresh,
      std::pair<double, double> forceGain, std::pair<double, double> torqueGain)
  : ComplianceTask(robots, robotIndex, forceSensor, ctlWrenches, calibrator,
      timestep, Eigen::Matrix6d::Identity(), stiffness, weight,
      forceThresh, torqueThresh, forceGain, torqueGain)
{
}

void ComplianceTask::addToSolver(mc_solver::QPSolver & solver)
{
  efTask_->addToSolver(solver);
}

void ComplianceTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  efTask_->removeFromSolver(solver);
}

sva::PTransformd ComplianceTask::computePose()
{
    Eigen::Vector3d trans = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    if(wrench_.force().norm() > forceThresh_)
    {
      trans = (forceGain_.first*wrench_.force() + forceGain_.second*errorD_.force()).unaryExpr(clampTrans_);
    }
    if(wrench_.couple().norm() > torqueThresh_)
    {
      Eigen::Vector3d rpy = (torqueGain_.first*wrench_.couple() + torqueGain_.second*errorD_.couple()).unaryExpr(clampRot_);
      rot = mc_rbdyn::rpyToMat(rpy);
    }
    const auto X_p_f = sensor_.X_p_f;
    const auto X_0_p = robot_.mbc().bodyPosW[robot_.bodyIndexByName(sensor_.parentBodyName)];
    sva::PTransformd move(rot, trans);
    auto X_f_ds = calibrator_.X_fsmodel_fsactual(sensor_.sensorName);
    return ((X_f_ds*X_p_f).inv()*move*(X_f_ds*X_p_f))*X_0_p;
}

void ComplianceTask::update()
{
  error_ = wrench_;
  /* Get wrench, remove gravity, use dof_ to deactivate some axis */
  wrench_ = ctlWrenches_.at(sensor_.sensorName);
  calibrator_.removeGravity(wrench_, sensor_.sensorName, robot_);
  wrench_ = sva::ForceVecd(dof_*(wrench_ - obj_).vector());
  errorD_ = (wrench_ - error_)/timestep_;
  efTask_->set_ef_pose(computePose());
  /* Does nothing for now, but is here in case of changes */
  efTask_->update();
}

void ComplianceTask::reset()
{
  efTask_->reset();
}

sva::ForceVecd ComplianceTask::getFilteredWrench() const
{
  return wrench_ + sva::ForceVecd(dof_*obj_.vector());
}

void ComplianceTask::dimWeight(const Eigen::VectorXd & dimW)
{
  efTask_->dimWeight(dimW);
}

Eigen::VectorXd ComplianceTask::dimWeight() const
{
  return efTask_->dimWeight();
}

void ComplianceTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                const std::vector<std::string> & activeJointsName)
{
  efTask_->selectActiveJoints(solver, activeJointsName);
}

void ComplianceTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                  const std::vector<std::string> & unactiveJointsName)
{
  efTask_->selectUnactiveJoints(solver, unactiveJointsName);
}

void ComplianceTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  efTask_->resetJointsSelector(solver);
}

} // mc_tasks
