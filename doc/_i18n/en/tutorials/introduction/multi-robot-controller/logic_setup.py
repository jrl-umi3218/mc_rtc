# Declare constants
APPROACH = 0
HANDLE = 1
OPEN = 2
# In constructor
self.phase = APPROACH
# New method for our controller
def switch_phase(self):
  if self.phase == APPROACH and False: # We write this condition later
    # Setup the HANDLE phase
    self.phase = HANDLE
  elif self.phase == HANDLE and False: # We write this condition later
    # Setup the OPEN phase
    self.phase = OPEN
# Call this in the run callback
def run_callback(self):
    self.switch_phase()
    return True
