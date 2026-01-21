// In the header
enum ControllerPhase
{
  IDLE = 0,
  STARTED,
  MOVE
};
// A private property of our controller
ControllerPhase phase = IDLE;
// In the run function
bool DualArmController::run()
{
  if(phase_ == IDLE && 0 /** we write this condition later */)
  {
    /** Setup the STARTED phase */
    phase_ = STARTED;
  }
  else if(phase_ == STARTED && 0 /** we write this condition later */)
  {
    /** Setup the MOVE phase */
    phase_ = MOVE;
  }
  else if(phase_ == MOVE && 0 /** we write this condition later */) {}
  return mc_control::MCController::run();
}
