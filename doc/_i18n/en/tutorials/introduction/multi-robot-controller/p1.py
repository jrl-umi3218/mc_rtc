# In the reset callback
# Create the task and add it to the solver
self.handTask = mc_tasks.SurfaceTransformTask("RightGripper", self.robots(), 0, 5.0, 1000.0)
self.qpsolver.addTask(self.handTask)
# Set a target relative to the handle position
self.handTask.target(sva.PTransformd(eigen.Vector3d(0, 0, -0.025)) * self.robots().robot(1).surfacePose("Handle"))
