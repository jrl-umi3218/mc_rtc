#include <mc_tasks/CompliantOrientationTask.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_tvm/Robot.h>
#include <RBDyn/Jacobian.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{

CompliantOrientationTask::CompliantOrientationTask(const std::string & bodyName_,
                                                   const mc_rbdyn::Robots & robots,
                                                   unsigned int robotIndex,
                                                   double stiffness,
                                                   double weight)
: OrientationTask(robots.robot(robotIndex).frame(bodyName_), stiffness, weight), Gamma_(Eigen::Matrix3d::Zero()),
  tvm_robot_(robots.robot(robotIndex).tvmRobot()), rIdx_(robotIndex), frame_(robots.robot(robotIndex).frame(bodyName_)),
  refAccel_(Eigen::Vector3d::Zero())
{
  if(backend_ != Backend::Tasks)
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks] Can't use CompliantEndEffectorTask with {} backend, please use Tasks backend", backend_);

  type_ = "compliant_position";
  name_ = std::string("compliant_position_") + frame_.robot().name() + "_" + frame_.name();
  OrientationTask::name(name_);
}

void CompliantOrientationTask::refAccel(const Eigen::Vector3d & refAccel) noexcept
{
  refAccel_ = refAccel;
}

void CompliantOrientationTask::update(mc_solver::QPSolver & solver)
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
    Eigen::Vector3d frame_acc = (J * acc).head(3);
    disturbance = Gamma_ * frame_acc;
  }
  else
  {
    disturbance.setZero();
  }
  // mc_rtc::log::info("Ref accel from disturbance : {}", disturbance.transpose());
  Eigen::Vector3d disturbedAccel = refAccel_ + disturbance;
  OrientationTask::refAccel(disturbedAccel);
  OrientationTask::update(solver);
}

void CompliantOrientationTask::makeCompliant(bool compliance)
{
  if(compliance) { Gamma_.diagonal().setOnes(); }
  else
  {
    Gamma_.diagonal().setZero();
  }
}

void CompliantOrientationTask::setComplianceVector(Eigen::Vector3d Gamma)
{
  Gamma_.diagonal() = Gamma;
}

bool CompliantOrientationTask::isCompliant(void)
{
  return Gamma_.diagonal().norm() > 0;
}

Eigen::Vector3d CompliantOrientationTask::getComplianceVector(void)
{
  return Gamma_.diagonal();
}

void CompliantOrientationTask::addToSolver(mc_solver::QPSolver & solver)
{
  OrientationTask::addToSolver(solver);
  jac_ = new rbd::Jacobian(robots.robot(rIdx_).mb(), frame_.body());
}

void CompliantOrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  OrientationTask::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_, "Compliance"},
      mc_rtc::gui::Checkbox(
          "Compliance is active", [this]() { return isCompliant(); }, [this]() { makeCompliant(!isCompliant()); }),
      mc_rtc::gui::ArrayInput("Gamma", {"x", "y", "z"}, Gamma_));
}

} // namespace mc_tasks
