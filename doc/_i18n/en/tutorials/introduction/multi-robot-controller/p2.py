# Modify switch_phase(self)
if self.phase == APPROACH and self.handTask.eval().norm() < 0.05 and self.handTask.speed().norm() < 1e-4:
  # Add a new contact
  self.addContact(self.robot().name(), "door", "RightGripper", "Handle")
  # Remove the surface transform task
  self.qpsolver.removeTask(self.handTask)
  # Keep the robot in its current posture
  self.postureTask.reset()
  self.comTask.reset()
  # Target new handle position
  self.doorPosture.target({"handle": {-1.0}})
  # Switch phase
  self.phase = HANDLE
