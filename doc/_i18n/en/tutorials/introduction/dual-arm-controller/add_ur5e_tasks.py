# In the reset callback
self._urEndEffectorTask = mc_tasks.EndEffectorTask("wrist_3_link", self.robots(), 0)
self._urEndEffectorTask.positionTask.stiffness(1)
self._urEndEffectorTask.orientationTask.stiffness(1)
