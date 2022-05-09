// Modify MyFirstController::switch_phase()
if(phase == APPROACH && handTask->eval().norm < 0.05 && handTask->speed().norm() < 1e-4)
{
  // Add a new contact
  addContact({robot().name(), "door", "RightGripper", "Handle"});
  // Remove the surface transform task
  solver().removeTask(handTask);
  // Keep the robot in its current posture
  postureTask->reset();
  comTask->reset();
  // Target new handle position
  doorPosture->target({{"handle", {-1.0}}});
  // Switch phase
  phase = HANDLE;
}
