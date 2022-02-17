// In the header
std::shared_ptr<mc_solver::KinematicsConstraint> doorKinematics;
std::shared_ptr<mc_tasks::PostureTask> doorPosture;
// In the reset function
doorKinematics = std::make_shared<mc_solver::KinematicsConstraint>(robots(), 1, solver().dt());
solver().addConstraintSet(*doorKinematics);
doorPosture = std::make_shared<mc_tasks::PostureTask>(solver(), 1, 5.0, 1000.0);
solver().addTask(doorPosture)
