# Import the mc_tasks module
import mc_tasks
# In the constructor, create the task and add it to the problem
self.efTask = mc_tasks.EndEffectorTask("l_wrist", self.robots(), 0, 10.0, 1000.0)
self.qpsolver.addTask(self.efTask)
# In the reset callback, reset the task to the current EF position
self.efTask.reset()
