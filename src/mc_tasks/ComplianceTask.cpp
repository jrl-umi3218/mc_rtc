/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

namespace force
{

namespace
{

std::function<double(double)> clamper(double value)
{
  auto clamp = [value](const double & v) { return std::min(std::max(v, -value), value); };
  return clamp;
}

} // namespace

ComplianceTask::ComplianceTask(const mc_rbdyn::Robots & robots,
                               unsigned int robotIndex,
                               const std::string & body,
                               double timestep,
                               const Eigen::Matrix6d & dof,
                               double stiffness,
                               double weight,
                               double forceThresh,
                               double torqueThresh,
                               std::pair<double, double> forceGain,
                               std::pair<double, double> torqueGain)
: wrench_(Eigen::Vector6d::Zero()), obj_(Eigen::Vector6d::Zero()), error_(Eigen::Vector6d::Zero()),
  errorD_(Eigen::Vector6d::Zero()), robots_(robots), rIndex_(robotIndex),
  sensor_(robots_.robot(rIndex_).bodyForceSensor(body)), timestep_(timestep), forceThresh_(forceThresh),
  torqueThresh_(torqueThresh), forceGain_(forceGain), torqueGain_(torqueGain), dof_(dof)
{
  efTask_ = std::make_shared<EndEffectorTask>(body, robots, robotIndex, stiffness, weight);
  clampTrans_ = clamper(0.01);
  clampRot_ = clamper(0.1);

  type_ = "compliance";
  name_ = "compliance_" + robots_.robot(rIndex_).name() + "_" + body;
}

ComplianceTask::ComplianceTask(const mc_rbdyn::Robots & robots,
                               unsigned int robotIndex,
                               const std::string & body,
                               double timestep,
                               double stiffness,
                               double weight,
                               double forceThresh,
                               double torqueThresh,
                               std::pair<double, double> forceGain,
                               std::pair<double, double> torqueGain)
: ComplianceTask(robots,
                 robotIndex,
                 body,
                 timestep,
                 Eigen::Matrix6d::Identity(),
                 stiffness,
                 weight,
                 forceThresh,
                 torqueThresh,
                 forceGain,
                 torqueGain)
{
}

void ComplianceTask::addToSolver(mc_solver::QPSolver & solver)
{
  MetaTask::addToSolver(*efTask_, solver);
}

void ComplianceTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*efTask_, solver);
}

sva::PTransformd ComplianceTask::computePose()
{
  Eigen::Vector3d trans = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  if(wrench_.force().norm() > forceThresh_)
  {
    trans = (forceGain_.first * wrench_.force() + forceGain_.second * errorD_.force()).unaryExpr(clampTrans_);
  }
  if(wrench_.couple().norm() > torqueThresh_)
  {
    Eigen::Vector3d rpy =
        (torqueGain_.first * wrench_.couple() + torqueGain_.second * errorD_.couple()).unaryExpr(clampRot_);
    rot = mc_rbdyn::rpyToMat(rpy);
  }
  const auto & X_p_f = sensor_.X_p_f();
  const auto & X_0_p =
      robots_.robot(rIndex_).mbc().bodyPosW[robots_.robot(rIndex_).bodyIndexByName(sensor_.parentBody())];
  sva::PTransformd move(rot, trans);
  const auto & X_f_ds = sensor_.X_fsmodel_fsactual();
  return ((X_f_ds * X_p_f).inv() * move * (X_f_ds * X_p_f)) * X_0_p;
}

void ComplianceTask::update(mc_solver::QPSolver & solver)
{
  error_ = wrench_;
  /* Get wrench, remove gravity, use dof_ to deactivate some axis */
  wrench_ = sensor_.wrenchWithoutGravity(robots_.robot(rIndex_));
  wrench_ = sva::ForceVecd(dof_ * (wrench_ - obj_).vector());
  errorD_ = (wrench_ - error_) / timestep_;
  efTask_->set_ef_pose(computePose());
  /* Does nothing for now, but is here in case of changes */
  MetaTask::update(*efTask_, solver);
}

void ComplianceTask::reset()
{
  efTask_->reset();
}

sva::ForceVecd ComplianceTask::getFilteredWrench() const
{
  return wrench_ + sva::ForceVecd(dof_ * obj_.vector());
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
                                        const std::vector<std::string> & activeJointsName,
                                        const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  efTask_->selectActiveJoints(solver, activeJointsName, activeDofs);
}

void ComplianceTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                          const std::vector<std::string> & unactiveJointsName,
                                          const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  efTask_->selectUnactiveJoints(solver, unactiveJointsName, unactiveDofs);
}

void ComplianceTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  efTask_->resetJointsSelector(solver);
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "compliance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      Eigen::Matrix6d dof = Eigen::Matrix6d::Identity();
      config("dof", dof);
      auto t = std::shared_ptr<mc_tasks::force::ComplianceTask>(
          new mc_tasks::force::ComplianceTask(solver.robots(), config("robotIndex"), config("body"), solver.dt(), dof));
      if(config.has("stiffness"))
      {
        t->stiffness(config("stiffness"));
      }
      if(config.has("weight"))
      {
        t->weight(config("weight"));
      }
      if(config.has("forceThresh"))
      {
        t->forceThresh(config("forceThresh"));
      }
      if(config.has("torqueThresh"))
      {
        t->torqueThresh(config("torqueThresh"));
      }
      if(config.has("forceGain"))
      {
        t->forceGain(config("forceGain"));
      }
      if(config.has("torqueGain"))
      {
        t->torqueGain(config("torqueGain"));
      }
      if(config.has("wrench"))
      {
        t->setTargetWrench(config("wrench"));
      }
      t->load(solver, config);
      return t;
    });
}
