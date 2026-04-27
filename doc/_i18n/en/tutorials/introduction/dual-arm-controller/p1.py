# In the run callback
def run_callback(self):
    if self._phase == ControllerPhase.IDLE:
        self.postureTask.target(
            {
                "elbow_joint".encode("utf-8"): [-math.pi / 2],
                "wrist_2_joint".encode("utf-8"): [math.pi / 2],
            }
        )
        self._phase = ControllerPhase.STARTED
    # ...
