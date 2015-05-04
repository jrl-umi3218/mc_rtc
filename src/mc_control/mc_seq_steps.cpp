#include <mc_control/mc_seq_steps.h>

namespace mc_control
{

bool enter_initT::eval(MCSeqController & controller)
{
  std::cout << "INIT" << std::endl;
  return true;
}

bool live_initT::eval(MCSeqController & controller)
{
  controller.updateRobotEnvCollisions(controller.stances[controller.stanceIndex], controller.configs[controller.stanceIndex]);
  controller.updateSelfCollisions(controller.stances[controller.stanceIndex], controller.configs[controller.stanceIndex]);
  controller.updateContacts(controller.stances[controller.stanceIndex].contacts());
  controller.updateSolverEqInEq();

  controller.stanceIndex += 1;

  return true;
}

bool live_chooseContactT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_removeContactT::eval(MCSeqController & controller)
{
  return false;
}

bool live_removeContacT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_moveWPT::eval(MCSeqController & controller)
{
  return false;
}

bool live_moveWPT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_moveContactP::eval(MCSeqController & controller)
{
  return false;
}

bool live_moveContactT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_pushContactT::eval(MCSeqController & controller)
{
  return false;
}

bool live_pushContactT::eval(MCSeqController & controller)
{
  return false;
}

bool live_chooseCoMT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_moveCoMP::eval(MCSeqController & controller)
{
  return false;
}

bool live_moveCoMT::eval(MCSeqController & controller)
{
  return false;
}

bool live_chooseGripperT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_openGripperP::eval(MCSeqController & controller)
{
  return false;
}

bool live_openGripperP::eval(MCSeqController & controller)
{
  return false;
}

bool live_openGripperT::eval(MCSeqController & controller)
{
  return false;
}

bool live_openGripperNotRmT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_removeGripperP::eval(MCSeqController & controller)
{
  return false;
}

bool live_removeGripperP::eval(MCSeqController & controller)
{
  return false;
}

bool live_removeGripperT::eval(MCSeqController & controller)
{
  return false;
}

bool live_removeGripperNotAddT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_moveGripperWPT::eval(MCSeqController & controller)
{
  return false;
}

bool live_moveGripperWPT::eval(MCSeqController & controller)
{
  return false;
}

bool live_moveGripperT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_adjustGripperP::eval(MCSeqController & controller)
{
  return false;
}

bool live_adjustGripperT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_addGripperT::eval(MCSeqController & controller)
{
  return false;
}

bool live_addGripperT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_closeGripperP::eval(MCSeqController & controller)
{
  return false;
}

bool live_closeGripperP::eval(MCSeqController & controller)
{
  return false;
}

bool live_closeGripperT::eval(MCSeqController & controller)
{
  return false;
}

bool live_closeGripperMoveCoMT::eval(MCSeqController & controller)
{
  return false;
}

bool enter_contactGripperP::eval(MCSeqController & controller)
{
  return false;
}

bool live_contactGripperT::eval(MCSeqController & controller)
{
  return false;
}

}
