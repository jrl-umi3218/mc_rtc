// In the header
std::shared_ptr<mc_tasks::PostureTask> kinovaPostureTask_;
std::shared_ptr<mc_solver::KinematicsConstraint> kinovaKinematics_;
// In the reset function
kinovaPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), 1);
solver().addTask(kinovaPostureTask_);
kinovaKinematics_ = std::make_shared<mc_solver::KinematicsConstraint>(robots(), 1, solver().dt());
solver().addConstraintSet(*kinovaKinematics_);
