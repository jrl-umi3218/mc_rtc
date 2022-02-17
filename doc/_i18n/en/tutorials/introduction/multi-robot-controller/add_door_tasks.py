# In the reset callback
self.robots().robot(1).posW(sva.PTransformd(sva.RotZ(math.pi), eigen.Vector3d(0.7, 0.5, 0)))
self.doorKinematics = mc_solver.KinematicsConstraint(self.robots(), 1, self.qpsolver.timeStep)
self.qpsolver.addConstraintSet(self.doorKinematics)
self.doorPosture = mc_tasks.PostureTask(self.qpsolver, 1, 5.0, 1000.0)
self.qpsolver.addTask(self.doorPosture)
