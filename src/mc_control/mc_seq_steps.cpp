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
  controller.updateRobotEnvCollisions(controller.stances[controller.stanceIndex].contacts(), controller.configs[controller.stanceIndex]);
  controller.updateSelfCollisions(controller.stances[controller.stanceIndex].contacts(), controller.configs[controller.stanceIndex]);
  controller.updateContacts(controller.stances[controller.stanceIndex].contacts());
  controller.updateSolverEqInEq();

  controller.stanceIndex += 1;

  return true;
}

bool live_chooseContactT::eval(MCSeqController & ctl)
{
  /* Will be stuck here if the contact is not supported */
  mc_rbdyn::StanceAction * curAction = &(ctl.curAction());
  mc_rbdyn::StanceAction * targetAction = &(ctl.targetAction());
  mc_rbdyn::RemoveContactAction * rmCurAction = dynamic_cast<mc_rbdyn::RemoveContactAction*>(curAction);
  mc_rbdyn::AddContactAction * addTargetAction = dynamic_cast<mc_rbdyn::AddContactAction*>(targetAction);
  bool isAddRemove = rmCurAction != 0 and addTargetAction != 0 and
                     rmCurAction->contact.robotSurface->bodyName == addTargetAction->contact.robotSurface->bodyName;
  if(isAddRemove and rmCurAction->contact.robotSurface->type() != "gripper")
  {
    ctl.currentContact = &(rmCurAction->contact);
    ctl.targetContact = &(addTargetAction->contact);
    return true;
  }
  bool isAddOnly = addTargetAction != 0;
  if(isAddOnly and addTargetAction->contact.robotSurface->type() != "gripper")
  {
    ctl.currentContact = 0;
    ctl.targetContact = &(addTargetAction->contact);
    std::string bodyName = ctl.targetContact->robotSurface->bodyName;
    /* FIXME Hard-coded */
    if(bodyName == "RARM_LINK6")
    {
      ctl.currentGripper = ctl.rgripper.get();
    }
    else if(bodyName == "LARM_LINK6")
    {
      ctl.currentGripper = ctl.lgripper.get();
    }
    else
    {
      ctl.currentGripper = 0;
    }
    return true;
  }

  std::cerr << "\rUnsupported contact passed into live_chooseContactT, stuck here..." << std::flush;
  return false;
}

bool enter_removeContactT::eval(MCSeqController & ctl)
{
  std::cout << "MOVECONTACT" << std::endl;
  if(ctl.currentContact == 0)
  {
    return true;
  }
  else
  {
    ctl.currentGripper = 0;
  }

  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  mc_rbdyn::Stance & curS = ctl.curStance();

  /* Remove contact */
  ctl.rmContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, *(ctl.currentContact), contactConf));
  ctl.rmContactTask->addToSolver(ctl.qpsolver->solver);
  ctl.metaTasks.push_back(ctl.rmContactTask.get());

  ctl.updateRobotEnvCollisions(curS.stabContacts, contactConf);
  ctl.updateSelfCollisions(curS.stabContacts, contactConf);
  ctl.updateContacts(curS.stabContacts);
  ctl.updateSolverEqInEq();

  ctl.isColl = ctl.setCollisionsContactFilter(*(ctl.currentContact), contactConf);
  ctl.notInContactCount = 0;

  return true;
}

bool live_removeContacT::eval(MCSeqController & ctl)
{
  if(ctl.currentContact == 0)
  {
    return true;
  }
  /*FIXME Need ContactSensor implementation from here */
  return false;
}

bool enter_moveWPT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_moveWPT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_moveContactP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_moveContactT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_pushContactT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_pushContactT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_chooseCoMT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_moveCoMP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_moveCoMT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_chooseGripperT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_openGripperP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_openGripperP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_openGripperT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_openGripperNotRmT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_removeGripperP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_removeGripperP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_removeGripperT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_removeGripperNotAddT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_moveGripperWPT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_moveGripperWPT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_moveGripperT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_adjustGripperP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_adjustGripperT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_addGripperT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_addGripperT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_closeGripperP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_closeGripperP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_closeGripperT::eval(MCSeqController & ctl)
{
  return false;
}

bool live_closeGripperMoveCoMT::eval(MCSeqController & ctl)
{
  return false;
}

bool enter_contactGripperP::eval(MCSeqController & ctl)
{
  return false;
}

bool live_contactGripperT::eval(MCSeqController & ctl)
{
  return false;
}

}
