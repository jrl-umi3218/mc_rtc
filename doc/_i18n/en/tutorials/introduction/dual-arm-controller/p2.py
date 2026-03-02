# In the run callback
def run_callback(self):
    # ...
    elif (
            self._phase == ControllerPhase.STARTED
            and self.postureTask.eval().norm() < 0.01
            and self.postureTask.speed().norm() < 0.01
        ):
            self._phase = ControllerPhase.MOVE
            self.qpsolver.addTask(self._urEndEffectorTask)
    elif self._phase == ControllerPhase.STARTED:
        self._urEndEffectorTask.reset()
    # ...
    return True