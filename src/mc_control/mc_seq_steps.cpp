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

  bool notInContact = ! ctl.inContact(ctl.currentContact->robotSurface->name);
  if(notInContact)
  {
    ctl.notInContactCount++;
  }

  if(ctl.notInContactCount > 50)
  {
    ctl.rmContactTask->removeFromSolver(ctl.qpsolver->solver);
    ctl.removeMetaTask(ctl.rmContactTask.get());
    ctl.rmContactTask.reset();

    if(ctl.isColl)
    {
      ctl.updateRobotEnvCollisions(ctl.curStance().stabContacts, ctl.curConf());
    }

    return true;
  }

  return false;
}

bool enter_moveWPT::eval(MCSeqController & ctl)
{
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  mc_rbdyn::Stance & newS = ctl.targetStance();

  /* Configure the stability task */
  ctl.stabilityTask->target(ctl.env(), newS, contactConf, contactConf.comTask.targetSpeed);

  /* Create and setup move contact task */
  ctl.moveContactTask.reset(new mc_tasks::MoveContactTask(ctl.robots(), *ctl.targetContact, contactConf, 0.5));
  ctl.moveContactTask->addToSolver(ctl.qpsolver->solver);
  ctl.moveContactTask->toWaypoint(contactConf, contactConf.contactTask.position.targetSpeed);
  ctl.metaTasks.push_back(ctl.moveContactTask.get());

  return true;
}

bool live_moveWPT::eval(MCSeqController & ctl)
{
  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen += 0.002;
    if(ctl.currentGripper->percentOpen >= 1)
    {
      ctl.currentGripper->percentOpen = 1;
    }
  }
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();

  double curOriWeight = ctl.moveContactTask->orientationTaskSp->weight();
  double tarOriWeight = contactConf.contactTask.orientation.finalWeight;
  ctl.moveContactTask->orientationTaskSp->weight(std::min(curOriWeight + 0.5, tarOriWeight));

  Eigen::Vector3d robotSurfacePos = ctl.moveContactTask->robotSurfacePos().translation();
  Eigen::Vector3d targetPos = ctl.moveContactTask->wp;
  double error = (robotSurfacePos - targetPos).norm();

  if(error < contactConf.contactTask.waypointConf.thresh)
  {
    ctl.moveContactTask->toPreEnv(contactConf, contactConf.contactTask.position.targetSpeed);
    if(ctl.currentGripper == 0)
    {
      return true;
    }
    else if(ctl.currentGripper->percentOpen >= 1)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }

  return false;
}

bool enter_moveContactP::eval(MCSeqController & ctl)
{
  ctl.isCollFiltered = false;
  ctl.contactSensor->resetOffset();
  return true;
}

bool live_moveContactT::eval(MCSeqController & ctl)
{
  auto & obj = ctl.curConf().contactObj;

  double curOriWeight = ctl.moveContactTask->orientationTaskSp->weight();
  double tarOriWeight = ctl.curConf().contactTask.orientation.finalWeight;
  ctl.moveContactTask->orientationTaskSp->weight(std::min(curOriWeight + 1.0, tarOriWeight));

  Eigen::Vector3d robotSurfacePos = ctl.moveContactTask->robotSurfacePos().translation();
  Eigen::VectorXd robotSurfaceVel = ctl.moveContactTask->robotSurfaceVel().vector();
  Eigen::Vector3d targetPos = ctl.moveContactTask->preTargetPos;
  double posErr = (targetPos - robotSurfacePos).norm();
  double velErr = robotSurfaceVel.norm();

  /* Remove collision avoidance when velocity reaches 0 to set the contact more precisely */
  if( (not ctl.isCollFiltered) and velErr < obj.velThresh )
  {
    ctl.setCollisionsContactFilter(*(ctl.targetContact), ctl.curConf());
    ctl.isCollFiltered = true;
  }

  bool inContact = ctl.inContact(ctl.targetContact->robotSurface->name);
  if( (posErr < obj.posThresh and velErr < obj.velThresh) or inContact )
  {
    ctl.moveContactTask->removeFromSolver(ctl.qpsolver->solver);
    ctl.removeMetaTask(ctl.moveContactTask.get());
    ctl.moveContactTask.reset();

    if(not ctl.isCollFiltered)
    {
      ctl.setCollisionsContactFilter(*(ctl.targetContact), ctl.curConf());
    }
    return true;
  }

  return false;
}

bool enter_pushContactT::eval(MCSeqController & ctl)
{
  ctl.push = false;
  bool inContact = ctl.inContact(ctl.targetContact->robotSurface->name);

  /* If not already in contact, move the contact in the normal direction */
  if(not inContact)
  {
    auto & contactConf = ctl.curConf();
    ctl.addContactTask.reset(new mc_tasks::AddContactTask(ctl.robots(), ctl.constSpeedConstr, *(ctl.targetContact), contactConf));
    ctl.addContactTask->addToSolver(ctl.qpsolver->solver);
    ctl.metaTasks.push_back(ctl.addContactTask.get());
    ctl.push = true;
  }

  return true;
}

bool live_pushContactT::eval(MCSeqController & ctl)
{
  bool inContact = ctl.inContact(ctl.targetContact->robotSurface->name);

  if(inContact)
  {
    /* Remove AddContactTask if we had it */
    if(ctl.push)
    {
      ctl.addContactTask->removeFromSolver(ctl.qpsolver->solver);
      ctl.removeMetaTask(ctl.addContactTask.get());
      ctl.addContactTask.reset();
    }
    ctl.currentContact = 0;
    ctl.targetContact = 0;

    mc_rbdyn::Stance & newS = ctl.targetStance();

    ctl.updateRobotEnvCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateSelfCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateContacts(newS.contacts());
    ctl.updateSolverEqInEq();

    ctl.stanceIndex++;
    return true;
  }
  else
  {
    return false;
  }
}

bool live_chooseCoMT::eval(MCSeqController & ctl)
{
  mc_rbdyn::StanceAction * curAction = &(ctl.curAction());
  mc_rbdyn::RemoveContactAction * rmCurAction = dynamic_cast<mc_rbdyn::RemoveContactAction*>(curAction);
  if(rmCurAction)
  {
    std::string bodyName = rmCurAction->contact.robotSurface->bodyName;
    if(bodyName == "RARM_LINK6") /*FIXME hard-coded */
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
  }

  mc_rbdyn::StanceAction * targetAction = &(ctl.targetAction());
  mc_rbdyn::AddContactAction * addTargetAction = dynamic_cast<mc_rbdyn::AddContactAction*>(targetAction);
  if(addTargetAction == 0)
  {
    return true;
  }

  std::cout << "\rStuck in live_chooseCoMT with wrong actions..." << std::flush;
  return false;
}

bool enter_moveCoMP::eval(MCSeqController & ctl)
{
  std::cout << "COMP" << std::endl;
  ctl.stabilityTask->target(ctl.env(), ctl.targetStance(), ctl.curConf(), ctl.curConf().comTask.targetSpeed);
  return true;
}

bool live_moveCoMT::eval(MCSeqController & ctl)
{
  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen -= 0.002;
    if(ctl.currentGripper->percentOpen <= 0)
    {
      ctl.currentGripper->percentOpen = 0;
    }
  }
  auto & obj = ctl.curConf().comObj;

  double error = (ctl.stabilityTask->comObj - rbd::computeCoM(*(ctl.robot().mb), *(ctl.robot().mbc))).norm();
  double errorVel = rbd::computeCoMVelocity(*(ctl.robot().mb), *(ctl.robot().mbc)).norm();

  if(error < obj.posThresh and errorVel < obj.velThresh)
  {
    ctl.stabilityTask->comObj = rbd::computeCoM(*(ctl.robot().mb), *(ctl.robot().mbc));
    ctl.stabilityTask->comTaskSm.reset(10, ctl.stabilityTask->comObj, 1);
    ctl.stabilityTask->postureTask->weight(100);
    ctl.stabilityTask->postureTask->posture(ctl.robot().mbc->q);

    mc_rbdyn::Stance & newS = ctl.targetStance();
    ctl.updateRobotEnvCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateSelfCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateContacts(newS.stabContacts);
    ctl.updateSolverEqInEq();

    ctl.stanceIndex++;
    return true;
  }

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
