#include <mc_tasks/CompliantPostureTask.h>

#include <mc_rtc/gui/Checkbox.h>
#include <mc_tvm/Robot.h>

namespace mc_tasks
{

CompliantPostureTask::CompliantPostureTask(const mc_solver::QPSolver & solver,
                                           unsigned int rIndex,
                                           double stiffness,
                                           double weight)
: PostureTask(solver, rIndex, stiffness, weight), isCompliant_(false),
  tvm_robot_(solver.robots().robot(rIndex).tvmRobot()),
  refAccel_(Eigen::VectorXd::Zero(solver.robots().robot(rIndex).mb().nrDof()))
{
  if(backend_ == Backend::Tasks)
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks] Can't use CompliantEndEffectorTask with {} backend, please use TVM or TVMHierarchical backend",
        backend_);
  name_ = "compliant_posture";
  type_ = "compliant_posture";
}

void CompliantPostureTask::refAccel(const Eigen::VectorXd & refAccel) noexcept
{
  refAccel_ = refAccel;
}

void CompliantPostureTask::update(mc_solver::QPSolver & solver)
{
  if(isCompliant_)
  {
    Eigen::VectorXd disturbance = tvm_robot_.alphaDExternal();
    // mc_rtc::log::info("Ref accel from disturbance : {}", disturbance.transpose());
    Eigen::VectorXd disturbedAccel = refAccel_ + disturbance;
    PostureTask::refAccel(disturbedAccel);
  }
  else { PostureTask::refAccel(refAccel_); }
  PostureTask::update(solver);
}

void CompliantPostureTask::makeCompliant(bool compliance)
{
  isCompliant_ = compliance;
}

bool CompliantPostureTask::isCompliant(void)
{
  return isCompliant_;
}

void CompliantPostureTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_, "Compliance"}, mc_rtc::gui::Checkbox(
                                                     "Compliance is active", [this]() { return isCompliant_; },
                                                     [this]() { isCompliant_ = !isCompliant_; }));
  PostureTask::addToGUI(gui);
}

} // namespace mc_tasks
