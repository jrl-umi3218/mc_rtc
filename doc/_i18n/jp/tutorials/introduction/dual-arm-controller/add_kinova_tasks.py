# resetコールバック関数内
self._kinovaPostureTask = mc_tasks.PostureTask(
    self.qpsolver, 1, 5.0, 1000.0
)
self._kinovaKinematicsConstraint = mc_solver.KinematicsConstraint(
    self.robots(), 1, self.qpsolver.dt()
)
self.qpsolver.addTask(self._kinovaPostureTask)
self.qpsolver.addConstraintSet(self._kinovaKinematicsConstraint)