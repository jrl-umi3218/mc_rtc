# Declare constants
class ControllerPhase:
    IDLE = 0
    STARTED = 1
    MOVE = 2


# In constructor
def __init__(self, rm, dt):
    self._phase = IDLE


# In the run callback
def run_callback(self):
    if self._phase == ControllerPhase.IDLE and False:  # We write this condition later
        # Setup the STARTED phase
        self._phase = ControllerPhase.STARTED
    elif (
        self._phase == ControllerPhase.STARTED and False
    ):  # We write this condition later
        # Setup the MOVE phase
        self._phase = ControllerPhase.MOVE
    elif self._phase == ControllerPhase.MOVE and False:  # We write this condition later
        # Continue the MOVE phase
        pass
    return True
