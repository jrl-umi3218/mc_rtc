elif self.phase == HANDLE and self.doorPosture.eval().norm() < 0.01:
  # Update door opening target
  self.doorPosture.target({"door": [0.5]})
  # Switch phase
  self.phase = OPEN
