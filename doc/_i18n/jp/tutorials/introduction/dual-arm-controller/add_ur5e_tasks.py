# resetコールバック関数内
self._urEndEffectorTask = mc_tasks.EndEffectorTask("tool0", self.robots(), 0)
self._urEndEffectorTask.positionTask.stiffness(1)
self._urEndEffectorTask.orientationTask.stiffness(1)
