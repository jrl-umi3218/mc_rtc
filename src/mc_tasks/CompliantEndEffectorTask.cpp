/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/CompliantEndEffectorTask.h>

#include <mc_rtc/gui/Checkbox.h>
#include <mc_tvm/Robot.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include "mc_rtc/gui/ArrayInput.h"

namespace mc_tasks
{

CompliantEndEffectorTask::CompliantEndEffectorTask(const std::string & bodyName,
                                                   const mc_rbdyn::Robots & robots,
                                                   unsigned int robotIndex,
                                                   double stiffness,
                                                   double weight)
: EndEffectorTask(robots.robot(robotIndex).frame(bodyName), stiffness, weight),
  compliant_matrix_(Eigen::Matrix6d::Zero()), tvm_robot_(nullptr), rIdx_(robotIndex), bodyName_(bodyName),
  refAccel_(Eigen::Vector6d::Zero())
{
  const mc_rbdyn::RobotFrame & frame = robots.robot(robotIndex).frame(bodyName);

  if(backend_ == Backend::Tasks)
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks] Can't use CompliantEndEffectorTask with {} backend, please use TVM or TVMHierarchical backend",
        backend_);

  type_ = "compliant_body6d";
  name_ = "compliant_body6d_" + frame.robot().name() + "_" + frame.name();
  EndEffectorTask::name(name_);
}

void CompliantEndEffectorTask::refAccel(const Eigen::Vector6d & refAccel) noexcept
{
  refAccel_ = refAccel;
}

void CompliantEndEffectorTask::makeCompliant(bool compliance)
{
  if(compliance) { compliant_matrix_.diagonal().setOnes(); }
  else { compliant_matrix_.diagonal().setZero(); }
}

void CompliantEndEffectorTask::setComplianceVector(Eigen::Vector6d gamma)
{
  compliant_matrix_.diagonal() = gamma;
}

bool CompliantEndEffectorTask::isCompliant(void)
{
  return compliant_matrix_.diagonal().norm() > 0;
}

Eigen::Vector6d CompliantEndEffectorTask::getComplianceVector(void)
{
  return compliant_matrix_.diagonal();
}

void CompliantEndEffectorTask::addToSolver(mc_solver::QPSolver & solver)
{
  tvm_robot_ = &solver.robots().robot(rIdx_).tvmRobot();
  jac_ = new rbd::Jacobian(tvm_robot_->robot().mb(), bodyName_);

  EndEffectorTask::addToSolver(solver);
}

void CompliantEndEffectorTask::update(mc_solver::QPSolver & solver)
{
  auto J = jac_->jacobian(tvm_robot_->robot().mb(), tvm_robot_->robot().mbc());
  Eigen::Vector6d disturbance = J * tvm_robot_->alphaDExternal();
  Eigen::Vector6d disturbedAccel = refAccel_ + compliant_matrix_ * disturbance;

  EndEffectorTask::positionTask->refAccel(disturbedAccel.tail(3));
  EndEffectorTask::orientationTask->refAccel(disturbedAccel.head(3));

  // EndEffectorTask::update(solver);
}

void CompliantEndEffectorTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_, "Compliance"}, mc_rtc::gui::Checkbox(
                                                     "Compliance is active", [this]() { return isCompliant(); },
                                                     [this]() { makeCompliant(!isCompliant()); }));
  gui.addElement({"Tasks", name_, "Compliance"},
                 mc_rtc::gui::ArrayInput(
                     "Compliance parameters", {"rx", "ry", "rz", "x", "y", "z"}, [this]()
                     { return getComplianceVector(); }, [this](Eigen::Vector6d v) { setComplianceVector(v); }));

  EndEffectorTask::addToGUI(gui);
}

} // namespace mc_tasks
