#include <mc_tasks/CompliantPostureTask.h>

#include <mc_rtc/gui/Checkbox.h>
#include <mc_tvm/Robot.h>
#include "mc_rtc/gui/ArrayInput.h"
#include "mc_rtc/logging.h"
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{

CompliantPostureTask::CompliantPostureTask(const mc_solver::QPSolver & solver,
                                           unsigned int rIndex,
                                           double stiffness,
                                           double weight)
: PostureTask(solver, rIndex, stiffness, weight),
  gamma_(Eigen::VectorXd::Zero(solver.robots().robot(rIndex).mb().nrDof())), robot_(solver.robots().robot(rIndex)),
  tvm_robot_(solver.robots().robot(rIndex).tvmRobot()),
  refAccel_(Eigen::VectorXd::Zero(solver.robots().robot(rIndex).mb().nrDof()))
{
  switch(backend_)
  {
    case Backend::Tasks:
    case Backend::TVM:
      break;
    default:
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "[mc_tasks] Can't use CompliantPostureTask with {} backend, please use Tasks or TVM backend", backend_);
      break;
  }
  name_ = std::string("compliant_posture_") + solver.robots().robot(rIndex).name();
  type_ = "compliant_posture";
}

void CompliantPostureTask::refAccel(const Eigen::VectorXd & refAccel) noexcept
{
  refAccel_ = refAccel;
}

void CompliantPostureTask::update(mc_solver::QPSolver & solver)
{
  Eigen::VectorXd disturbance;
  Eigen::VectorXd acc;
  if(backend_ == Backend::Tasks)
  {
    if(robot_.compensationTorquesAcc()) { acc = robot_.compensationTorquesAcc().value(); }
    else
    {
      acc = robot_.externalTorquesAcc();
    }
  }
  else
  {
    if(tvm_robot_.alphaDCompensation()) { acc = tvm_robot_.alphaDCompensation().value(); }
    else
    {
      acc = tvm_robot_.alphaDExternal();
    }
  }
  disturbance = gamma_.asDiagonal() * acc;

  // mc_rtc::log::info("Ref accel from disturbance : {}", disturbance.transpose());
  // mc_rtc::log::info("Ref accel : {}", refAccel_.transpose());
  Eigen::VectorXd disturbedAccel = refAccel_ + disturbance;
  PostureTask::refAccel(disturbedAccel);
  PostureTask::update(solver);
}

void CompliantPostureTask::makeCompliant(bool compliance)
{
  if(compliance) { gamma_.setOnes(); }
  else
  {
    gamma_.setZero();
  }
}

void CompliantPostureTask::makeCompliant(Eigen::VectorXd gamma)
{
  gamma_ = gamma;
}

bool CompliantPostureTask::isCompliant(void)
{
  return gamma_.norm() > 0;
}

void CompliantPostureTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  PostureTask::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_, "Compliance"},
      mc_rtc::gui::Checkbox(
          "Compliance is active", [this]() { return isCompliant(); }, [this]() { makeCompliant(!isCompliant()); }),
      mc_rtc::gui::ArrayInput("Gamma", {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5", "Joint_6", "Joint_7"},
                              gamma_));
}

} // namespace mc_tasks
