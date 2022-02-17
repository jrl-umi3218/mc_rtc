# Import the mc_tasks module
import mc_tasks
# In the constructor, create the task and add it to the problem
self.comTask = mc_tasks.CoMTask(self.robots(), 0, 10.0, 1000.0)
self.qpsolver.addTask(self.comTask)
# Reduce the posture task stiffness
self.postureTask.stiffness(1)
# In the reset callback, reset the task to the current CoM
self.comTask.reset()
