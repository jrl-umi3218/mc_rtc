// In the header
enum DoorPhase
{
      APPROACH = 0,
      HANDLE,
      OPEN
};
// A private property of our controller
DoorPhase phase = APPROACH;
// A new method for our controller
void switch_phase()
{
  if(phase == APPROACH && 0 /** we write this condition later */)
  {
    /** Setup the HANDLE phase */
    phase = HANDLE;
  }
  else if(phase == HANDLE && 0 /** we write this condition later */)
  {
    /** Setup the OPEN phase */
    phase = OPEN;
  }
}
// Call this in the run function
bool MyFirstController::run()
{
  switch_phase();
  return mc_control::MCController::run();
}
