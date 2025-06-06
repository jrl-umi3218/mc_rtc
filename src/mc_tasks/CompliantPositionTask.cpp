#include <mc_tasks/CompliantPositionTask.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tvm/Robot.h>
#include <RBDyn/Jacobian.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{

CompliantPositionTask::CompliantPositionTask(const std::string & bodyName_,
                                             const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             double stiffness,
                                             double weight)
: PositionTask(robots.robot(robotIndex).frame(bodyName_), stiffness, weight), Gamma_(Eigen::Matrix3d::Zero()),
  tvm_robot_(robots.robot(robotIndex).tvmRobot()), rIdx_(robotIndex), frame_(robots.robot(robotIndex).frame(bodyName_)),
  refAccel_(Eigen::Vector3d::Zero())
{
  if(backend_ != Backend::Tasks)
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks] Can't use CompliantEndEffectorTask with {} backend, please use Tasks backend", backend_);

  type_ = "compliant_position";
  name_ = std::string("compliant_position_") + frame_.robot().name() + "_" + frame_.name();
  PositionTask::name(name_);
}

void CompliantPositionTask::refAccel(const Eigen::Vector3d & refAccel) noexcept
{
  refAccel_ = refAccel;
}

void CompliantPositionTask::update(mc_solver::QPSolver & solver)
{
  auto J = jac_->jacobian(robots.robot(rIndex).mb(), robots.robot(rIndex).mbc());
  Eigen::Vector3d disturbance;
  Eigen::VectorXd acc;
  if(backend_ == Backend::Tasks)
  {
    if(solver.robot().compensationTorquesAcc()) { acc = solver.robot().compensationTorquesAcc().value(); }
    else
    {
      acc = solver.robot().externalTorquesAcc();
    }
    Eigen::Vector3d frame_acc = (J * acc).tail(3);
    disturbance = Gamma_ * frame_acc;
  }
  else
  {
    disturbance.setZero();
  }
  // mc_rtc::log::info("Ref accel from disturbance : {}", disturbance.transpose());
  Eigen::Vector3d disturbedAccel = refAccel_ + disturbance;
  PositionTask::refAccel(disturbedAccel);
  PositionTask::update(solver);
}

void CompliantPositionTask::makeCompliant(bool compliance)
{
  if(compliance) { Gamma_.diagonal().setOnes(); }
  else
  {
    Gamma_.diagonal().setZero();
  }
}

void CompliantPositionTask::setComplianceVector(Eigen::Vector3d Gamma)
{
  Gamma_.diagonal() = Gamma;
}

bool CompliantPositionTask::isCompliant(void)
{
  return Gamma_.diagonal().norm() > 0;
}

Eigen::Vector3d CompliantPositionTask::getComplianceVector(void)
{
  return Gamma_.diagonal();
}

void CompliantPositionTask::addToSolver(mc_solver::QPSolver & solver)
{
  PositionTask::addToSolver(solver);
  jac_ = new rbd::Jacobian(robots.robot(rIdx_).mb(), frame_.body());
}

void CompliantPositionTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  PositionTask::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_, "Compliance"},
      mc_rtc::gui::Checkbox(
          "Compliance is active", [this]() { return isCompliant(); }, [this]() { makeCompliant(!isCompliant()); }),
      mc_rtc::gui::ArrayInput("Gamma", {"x", "y", "z"}, Gamma_));
}

} // namespace mc_tasks
