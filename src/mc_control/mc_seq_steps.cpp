#include <mc_control/mc_seq_steps.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include "pdgains.cpp"

#include <mc_rbdyn/Surface.h>

namespace mc_control
{

bool enter_initT::eval(MCSeqController &)
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
                     rmCurAction->contact.r1Surface->bodyName() == addTargetAction->contact.r1Surface->bodyName();
  if(isAddRemove and rmCurAction->contact.r1Surface->type() != "gripper")
  {
    ctl.currentContact = &(rmCurAction->contact);
    ctl.targetContact = &(addTargetAction->contact);
    return true;
  }
  bool isAddOnly = addTargetAction != 0;
  if(isAddOnly and addTargetAction->contact.r1Surface->type() != "gripper")
  {
    ctl.currentContact = 0;
    ctl.targetContact = &(addTargetAction->contact);
    std::string bodyName = ctl.targetContact->r1Surface->bodyName();
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

  bool notInContact = ! ctl.inContact(ctl.currentContact->r1Surface->name());
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
  ctl.moveContactTask.reset(new mc_tasks::MoveContactTask(ctl.robots(), ctl.robot(), ctl.env(), *ctl.targetContact, contactConf, 0.5));
  ctl.moveContactTask->addToSolver(ctl.qpsolver->solver);
  ctl.moveContactTask->toWaypoint(contactConf, contactConf.contactTask.position.targetSpeed);
  ctl.metaTasks.push_back(ctl.moveContactTask.get());

  return true;
}

bool live_moveWPT::eval(MCSeqController & ctl)
{
  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen += 0.0005;
    if(ctl.currentGripper->percentOpen >= 1)
    {
      ctl.currentGripper->percentOpen = 1;
      ctl.currentGripper = 0;
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
      std::cout << "Finished to move to contact wp" << std::endl;
      return true;
    }
    else if(ctl.currentGripper->percentOpen >= 1)
    {
      std::cout << "Finished to move to contact wp" << std::endl;
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

  //mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  //Eigen::Vector3d curSCoM = ctl.stances[ctl.stanceIndex].com(ctl.robot());
  //std::cout << "Current stance com" << std::endl;
  //std::cout << curSCoM << std::endl;
  //std::cout << "Next stance com" << std::endl;
  //Eigen::Vector3d tarSCoM = ctl.stances[ctl.stanceIndex+1].com(ctl.robot());
  //std::cout << tarSCoM << std::endl;
  //contactConf.comObj.comOffset = (tarSCoM - curSCoM);
  //contactConf.comObj.comOffset(0) /= 3;
  //contactConf.comObj.comOffset(1) /= 2;
  //contactConf.comObj.comOffset(2) /= 3;
  //std::cout << "Offset" << std::endl;
  //std::cout << contactConf.comObj.comOffset << std::endl;
  //ctl.stabilityTask->target(ctl.env(), ctl.stances[ctl.stanceIndex], contactConf, contactConf.comTask.targetSpeed);

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

  bool inContact = ctl.inContact(ctl.targetContact->r1Surface->name());
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
  bool inContact = ctl.inContact(ctl.targetContact->r1Surface->name());

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
  bool inContact = ctl.inContact(ctl.targetContact->r1Surface->name());

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
  ctl.comRemoveGripper = false;
  mc_rbdyn::StanceAction * currentAction = &(ctl.curAction());
  if(currentAction->type() == "remove")
  {
    mc_rbdyn::RemoveContactAction * rmCurAction = dynamic_cast<mc_rbdyn::RemoveContactAction*>(currentAction);
    std::string bodyName = rmCurAction->contact.r1Surface->bodyName();
    if(bodyName == "RARM_LINK6") /*FIXME hard-coded */
    {
      ctl.currentGripper = ctl.rgripper.get();
      ctl.currentGripperIsClosed = ctl.currentGripper->percentOpen < 0.5;
    }
    else if(bodyName == "LARM_LINK6")
    {
      ctl.currentGripper = ctl.lgripper.get();
      ctl.currentGripperIsClosed = ctl.currentGripper->percentOpen < 0.5;
    }
  }
  mc_rbdyn::StanceAction * targetAction = &(ctl.targetAction());
  if(targetAction->type() == "remove")
  {
    std::cout << "This action is a removal" << std::endl;
    mc_rbdyn::RemoveContactAction * rmCurAction = dynamic_cast<mc_rbdyn::RemoveContactAction*>(targetAction);
    std::string bodyName = rmCurAction->contact.r1Surface->bodyName();
    if(bodyName == "RARM_LINK6") /*FIXME hard-coded */
    {
      ctl.currentGripper = ctl.rgripper.get();
      ctl.currentGripperIsClosed = ctl.currentGripper->percentOpen < 0.5;
    }
    else if(bodyName == "LARM_LINK6")
    {
      ctl.currentGripper = ctl.lgripper.get();
      ctl.currentGripperIsClosed = ctl.currentGripper->percentOpen < 0.5;
    }
    if((bodyName == "RARM_LINK6" || bodyName == "LARM_LINK6") && ctl.actions[ctl.stanceIndex+1]->type() != "add")
    {
      std::cout << "Will move gripper away from contact in CoM" << std::endl;
      ctl.comRemoveGripper = true;
    }
  }

  if(targetAction->type() != "add")
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
  /*FIXME Should be optionnal */
  //ctl.stabilityTask->highStiffness({"RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6"});
  return true;
}

bool live_CoMOpenGripperT::eval(MCSeqController & ctl)
{
  if(ctl.currentGripper && ctl.currentGripperIsClosed && ctl.comRemoveGripper)
  {
    ctl.currentGripper->percentOpen += 0.0005;
    if(ctl.currentGripper->percentOpen >= 1)
    {
      ctl.currentGripper->percentOpen = 1;
      return true;
    }
  }
  else
  {
    return true;
  }
  return false;
}

bool enter_CoMRemoveGripperT::eval(MCSeqController & ctl)
{
  if(ctl.comRemoveGripper)
  {
    std::cout << "CoMRemoveGripperT" << std::endl;
    mc_rbdyn::StanceConfig & contactConf = ctl.targetConf();

    /* Find the contact to remove */
    mc_rbdyn::Contact & removedContact = *(ctl.targetContact);

    /* Create the remove contact meta task */
    ctl.removeContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, removedContact, contactConf));
    ctl.removeContactTask->addToSolver(ctl.qpsolver->solver);
    ctl.metaTasks.push_back(ctl.removeContactTask.get());

    /* We take the hull of the environment and the robot to know when the robot gripper is totally off */
    ctl.distPairs.clear();
    auto v = ctl.collisionsContactFilterList(removedContact, contactConf);
    for(const auto & vi :v )
    {
      std::shared_ptr<CollisionPair> ptr(new CollisionPair(ctl.robot(), ctl.env(), vi.first, vi.second));
      ctl.distPairs.push_back(ptr);
    }
    mc_rbdyn::Stance & targetS = ctl.targetStance();
    /* Configure the QP */
    ctl.updateRobotEnvCollisions(targetS.stabContacts, contactConf);
    ctl.updateSelfCollisions(targetS.stabContacts, contactConf);
    ctl.updateContacts(targetS.stabContacts);
    ctl.updateSolverEqInEq();

    ctl.isRemoved = false;
  }
  return true;
}


bool live_CoMRemoveGripperT::eval(MCSeqController & ctl)
{
  if(ctl.comRemoveGripper)
  {
    bool all = true;
    for(const auto & p : ctl.distPairs)
    {
      all = all && (p->distance(ctl.robot(), ctl.env()) > 0.05*0.05);
    }
    if(all)
    {
      ctl.removeContactTask->removeFromSolver(ctl.qpsolver->solver);
      ctl.removeMetaTask(ctl.removeContactTask.get());
      ctl.removeContactTask.reset();
      ctl.distPairs.clear();

      ctl.isRemoved = true;
      return true;
    }
    return false;
  }
  else
  {
    return true;
  }
}

bool live_moveCoMT::eval(MCSeqController & ctl)
{
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

    //ctl.stabilityTask->normalStiffness({"RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6"});

    ctl.stanceIndex++;
    return true;
  }

  return false;
}

bool live_CoMCloseGripperT::eval(MCSeqController & ctl)
{
  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen -= 0.0005;
    if(ctl.currentGripper->overCommandLimit || ctl.currentGripper->percentOpen <= 0)
    {
      if(!ctl.currentGripper->overCommandLimit)
      {
        ctl.currentGripper->percentOpen = 0;
      }
      ctl.currentGripper = 0;
      ctl.stanceIndex++;
      return true;
    }
    return false;
  }
  else
  {
    ctl.stanceIndex++;
    return true;
  }
}

bool live_chooseGripperT::eval(MCSeqController & ctl)
{
  mc_rbdyn::RemoveContactAction * curRm = dynamic_cast<mc_rbdyn::RemoveContactAction*>(&(ctl.curAction()));
  mc_rbdyn::AddContactAction * tarAdd = dynamic_cast<mc_rbdyn::AddContactAction*>(&(ctl.targetAction()));
  mc_rbdyn::RemoveContactAction * tarRm = dynamic_cast<mc_rbdyn::RemoveContactAction*>(&(ctl.targetAction()));
  bool isRemoveAdd = (curRm != 0) and (tarAdd != 0);
  if(isRemoveAdd)
  {
    isRemoveAdd = isRemoveAdd and (curRm->contact.r1Surface->bodyName() == tarAdd->contact.r1Surface->bodyName());
  }
  bool isAddOnly = tarAdd != 0;
  bool isRemoveOnly = tarRm != 0;
  if( (isRemoveAdd or isAddOnly) and tarAdd->contact.r1Surface->type() == "gripper" )
  {
    if(isRemoveAdd)
    {
      ctl.isGripperAttached = true;
      ctl.isGripperWillBeAttached = true;
      ctl.currentContact = &(curRm->contact);
      ctl.targetContact = &(tarAdd->contact);
    }
    else
    {
      ctl.isGripperAttached = false;
      ctl.isGripperWillBeAttached = true;
      ctl.currentContact = &(tarAdd->contact);
      ctl.targetContact = &(tarAdd->contact);
    }
    return true;
  }
  if(isRemoveOnly and tarRm->contact.r1Surface->type() == "gripper")
  {
    ctl.isGripperAttached = true;
    ctl.isGripperWillBeAttached = false;
    ctl.currentContact = &(tarRm->contact);
    ctl.targetContact = &(tarRm->contact);
    return true;
  }

  std::cout << "\rStuck in chooseGripperT, cannot handle the situation" << std::flush;
  return false;
}

bool enter_openGripperP::eval(MCSeqController & ctl)
{
  mc_rbdyn::Surface & robSurf = *(ctl.currentContact->r1Surface.get());
  std::string bodyName = robSurf.bodyName();

  /* If the gripper is an acti gripper, we reset the position */
  if(ctl.actiGrippers.count(bodyName))
  {
    ActiGripper & actiGrip = ctl.actiGrippers[bodyName];
    actiGrip.toRemove = true;
  }

  ctl.isGripperOpen = false;

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

bool live_openGripperP::eval(MCSeqController & ctl)
{
  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen += 0.0005;
    if(ctl.currentGripper->percentOpen >= 1.)
    {
      ctl.currentGripper->percentOpen = 1.;
      ctl.isGripperOpen = true;
      return true;
    }
  }
  else
  {
    ctl.isGripperOpen = true;
    return true;
  }
  return false;
}

bool live_openGripperT::eval(MCSeqController & ctl)
{
  return ctl.isGripperAttached and ctl.isGripperOpen;
}

bool live_openGripperNotRmT::eval(MCSeqController & ctl)
{
  return (not ctl.isGripperAttached) and ctl.isGripperOpen;
}

bool enter_removeGripperP::eval(MCSeqController & ctl)
{
  ctl.skipRemoveGripper = (not ctl.isGripperAttached) and ctl.isGripperOpen;
  if(ctl.skipRemoveGripper)
  {
    return true;
  }
  std::cout << "RemoveGripperP" << std::endl;
  mc_rbdyn::StanceConfig & contactConf = (ctl.currentContact == ctl.targetContact) ? ctl.targetConf() : ctl.curConf();

  /* Find the contact to remove */
  mc_rbdyn::Contact & removedContact = *(ctl.currentContact);

  /* Create the remove contact meta task */
  ctl.removeContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, removedContact, contactConf));
  ctl.removeContactTask->addToSolver(ctl.qpsolver->solver);
  ctl.metaTasks.push_back(ctl.removeContactTask.get());

  /* We take the hull of the environment and the robot to know when the robot gripper is totally off */
  ctl.distPairs.clear();
  auto v = ctl.collisionsContactFilterList(*(ctl.currentContact), contactConf);
  for(const auto & vi :v )
  {
    std::shared_ptr<CollisionPair> ptr(new CollisionPair(ctl.robot(), ctl.env(), vi.first, vi.second));
    ctl.distPairs.push_back(ptr);
  }
  mc_rbdyn::Stance & curS = (ctl.currentContact == ctl.targetContact) ? ctl.targetStance() : ctl.curStance();

  /* Configure the stability task */
  ctl.stabilityTask->target(ctl.env(), curS, contactConf, contactConf.comTask.targetSpeed);

  /* Configure the QP */
  ctl.updateRobotEnvCollisions(curS.stabContacts, contactConf);
  ctl.updateSelfCollisions(curS.stabContacts, contactConf);
  ctl.updateContacts(curS.stabContacts);
  ctl.updateSolverEqInEq();

  /* Remove collision avoidance between env and moving body */
  ctl.isColl = ctl.setCollisionsContactFilter(*(ctl.currentContact), contactConf);

  ctl.isRemoved = false;

  return true;
}

bool live_removeGripperP::eval(MCSeqController & ctl)
{
  if(ctl.skipRemoveGripper)
  {
    return true;
  }
  bool all = true;
  double dOut = ((not ctl.isGripperWillBeAttached) and ctl.isRemoved) ? 0.02 : 0.05;
  for(const auto & p : ctl.distPairs)
  {
    all = all && (p->distance(ctl.robot(), ctl.env()) > dOut*dOut);
  }
  if(all)
  {
    ctl.removeContactTask->removeFromSolver(ctl.qpsolver->solver);
    ctl.removeMetaTask(ctl.removeContactTask.get());
    ctl.removeContactTask.reset();
    ctl.distPairs.clear();

    if(ctl.isColl)
    {
      ctl.updateRobotEnvCollisions(ctl.curStance().stabContacts, ctl.curConf());
    }
    ctl.isRemoved = true;
    return true;
  }

  return false;
}

bool live_removeGripperT::eval(MCSeqController & ctl)
{
  return ctl.isGripperWillBeAttached and ctl.isRemoved;
}

bool live_removeGripperNotAddT::eval(MCSeqController & ctl)
{
  return (not ctl.isGripperWillBeAttached) and ctl.isRemoved;
}

bool enter_moveGripperWPT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  std::cout << "Move gripper WPT" << std::endl;
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  mc_rbdyn::Stance & newS = ctl.targetStance();

  /* Craete and setup move contact task */
  ctl.moveContactTask.reset(new mc_tasks::MoveContactTask(ctl.robots(), ctl.robot(), ctl.env(), *(ctl.targetContact), contactConf));
  ctl.moveContactTask->addToSolver(ctl.qpsolver->solver);
  ctl.moveContactTask->toWaypoint(contactConf, contactConf.contactTask.position.targetSpeed);
  ctl.metaTasks.push_back(ctl.moveContactTask.get());

  /* Configure the stability task */
  ctl.stabilityTask->target(ctl.env(), newS, contactConf, contactConf.comTask.targetSpeed);

  ctl.isBodyTask = false;
  if(ctl.curStance().contacts().size() <= 2)
  {
    unsigned int bodyIndex = ctl.robot().bodyIndexByName("BODY");
    ctl.bodyOriTask.reset(new tasks::qp::OrientationTask(ctl.robots().mbs, 0, ctl.robot().bodyIdByName("BODY"), ctl.robot().mbc->bodyPosW[bodyIndex].rotation()));
    ctl.bodyOriTaskSp.reset(new tasks::qp::SetPointTask(ctl.robots().mbs, 0, ctl.bodyOriTask.get(), 10, 1000));
    ctl.qpsolver->solver.addTask(ctl.bodyOriTaskSp.get());
    ctl.isBodyTask = true;
  }

  return true;
}

bool live_moveGripperWPT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();

  Eigen::Vector3d robotSurfacePos = ctl.moveContactTask->robotSurfacePos().translation();
  Eigen::Vector3d targetPos = ctl.moveContactTask->wp;
  double error = (robotSurfacePos - targetPos).norm();

  if(error < contactConf.contactTask.waypointConf.thresh)
  {
    /* Waypoint reached, new goal is target */
    ctl.moveContactTask->toPreEnv(contactConf, contactConf.contactTask.position.targetSpeed);
    std::cout << "Finished move gripper WPT" << std::endl;
    return true;
  }
  else
  {
    return false;
  }
}

bool live_moveGripperT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  mc_rbdyn::StanceConfig::ContactObj & obj = ctl.curConf().contactObj;

  Eigen::Vector3d robotSurfacePos = ctl.moveContactTask->robotSurfacePos().translation();
  Eigen::Vector3d targetPos = ctl.moveContactTask->preTargetPos;
  double posErr = (robotSurfacePos - targetPos).norm();
  double velErr = ctl.moveContactTask->robotSurfaceVel().linear().norm();

  if(ctl.isBodyTask)
  {
    double curBodyOriWeight = ctl.bodyOriTaskSp->weight();
    ctl.bodyOriTaskSp->weight(std::max(0.0, curBodyOriWeight - 0.5));
  }

  if(posErr < obj.posThresh and velErr < obj.velThresh)
  {
    if(ctl.isBodyTask)
    {
      ctl.qpsolver->solver.removeTask(ctl.bodyOriTaskSp.get());
    }
    std::cout << "Finished move gripper T" << std::endl;
    return true;
  }

  return false;
}

bool enter_adjustGripperP::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  std::cout << "Adjust Gripper P" << std::endl;
  double stiff = ctl.moveContactTask->posStiff + ctl.moveContactTask->extraPosStiff;
  ctl.adjustPositionTask.reset(new tasks::qp::PositionTask(ctl.robots().mbs, 0, ctl.moveContactTask->robotBodyId,
                                                          ctl.moveContactTask->positionTask->position(),
                                                          ctl.moveContactTask->robotSurf->X_b_s().translation()));
  ctl.adjustOrientationTask.reset(new tasks::qp::OrientationTask(ctl.robots().mbs, 0, ctl.moveContactTask->robotBodyId, ctl.moveContactTask->targetOri));
  ctl.adjustPositionTaskPid.reset(new tasks::qp::PIDTask(ctl.robots().mbs, 0, ctl.adjustPositionTask.get(), stiff, 4, 2*std::sqrt(stiff), 0));
  double oriStiff = ctl.moveContactTask->orientationTaskSp->stiffness();
  ctl.adjustOrientationTaskPid.reset(new tasks::qp::PIDTask(ctl.robots().mbs, 0, ctl.adjustOrientationTask.get(), oriStiff, 0.5, 2*std::sqrt(oriStiff), 0));

  Eigen::Vector3d error = ctl.moveContactTask->preTargetPos - ctl.moveContactTask->robotSurfacePos().translation();
  sva::MotionVecd M_0_s = ctl.moveContactTask->robotSurfaceVel();
  Eigen::VectorXd velModel = M_0_s.linear();

  ctl.adjustPositionTaskPid->error(error);
  ctl.adjustPositionTaskPid->errorI(Eigen::Vector3d(0,0,0));
  ctl.adjustPositionTaskPid->errorD(velModel);
  Eigen::Vector3d TBNWeight = ctl.curConf().contactObj.adjustOriTBNWeight;
  Eigen::Vector3d T = ctl.moveContactTask->targetTf.rotation().row(0)*TBNWeight[0];
  Eigen::Vector3d B = ctl.moveContactTask->targetTf.rotation().row(1)*TBNWeight[1];
  Eigen::Vector3d N = ctl.moveContactTask->targetTf.rotation().row(2)*TBNWeight[2];
  Eigen::Vector3d TBN = T + B + N;
  TBN[0] = std::abs(TBN[0]);
  TBN[1] = std::abs(TBN[1]);
  TBN[2] = std::abs(TBN[2]);
  ctl.adjustPositionTaskPid->dimWeight(TBN);
  ctl.errorI = Eigen::Vector3d(0,0,0);
  ctl.qpsolver->solver.addTask(ctl.adjustPositionTaskPid.get());

  Eigen::Vector3d oriModelError = sva::rotationError(ctl.robot().mbc->bodyPosW[ctl.moveContactTask->robotBodyIndex].rotation(),
                                                     ctl.moveContactTask->targetOri, 1e-7);
  Eigen::Vector3d oriModelVel = M_0_s.angular();
  ctl.adjustOrientationTaskPid->error(oriModelError);
  ctl.adjustOrientationTaskPid->errorI(Eigen::Vector3d(0,0,0));
  ctl.adjustOrientationTaskPid->errorD(oriModelVel);
  ctl.oriErrorI = Eigen::Vector3d(0,0,0);
  ctl.qpsolver->solver.addTask(ctl.adjustOrientationTaskPid.get());

  /* Don't manager positionTask with a smooth task anymore */
  ctl.moveContactTask->useSmoothTask = false;

  /* Remove collision detection between env body and robot body */
  ctl.setCollisionsContactFilter(*(ctl.targetContact), ctl.curConf());

  /*TODO This is where the manual control of the EF should go */
  ctl.isAdjust = true;

  ctl.contactSensor->resetOffset();

  return true;
}

bool live_adjustGripperT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  unsigned int bodyIndex = ctl.moveContactTask->robotBodyIndex;

  sva::PTransformd X_0_b = ctl.robot().mbc->bodyPosW[bodyIndex];
  sva::PTransformd X_0_s = ctl.moveContactTask->robotSurfacePos();
  sva::MotionVecd M_0_s = ctl.moveContactTask->robotSurfaceVel();
  Eigen::VectorXd velModel = M_0_s.linear();
  Eigen::Vector3d error = ctl.moveContactTask->preTargetPos - X_0_s.translation();

  Eigen::Vector3d oriModelError = sva::rotationError(X_0_b.rotation(),
                                                     ctl.moveContactTask->targetOri, 1e-7);
  Eigen::Vector3d oriModelVel = M_0_s.angular();

  if(ctl.isAdjust)
  {
    ctl.errorI -= error*ctl.timeStep; /*FIXME In python this use springError and robotEsti but since we don't use springs anyway... */
    ctl.oriErrorI -= oriModelError*ctl.timeStep;
  }
  else
  {
    /*FIXME Python use efcontrol here, we don't... */
  }

  auto antiWindup = [](Eigen::Vector3d & vec, double thresh)
  {
    vec[0] = std::min(std::max(vec[0], -thresh), thresh);
    vec[1] = std::min(std::max(vec[1], -thresh), thresh);
    vec[2] = std::min(std::max(vec[2], -thresh), thresh);
  };

  antiWindup(ctl.errorI, 0.5);
  antiWindup(ctl.oriErrorI, M_PI/2);

  ctl.adjustPositionTaskPid->error(error);
  ctl.adjustPositionTaskPid->errorI(ctl.errorI);
  ctl.adjustPositionTaskPid->errorD(velModel);

  ctl.adjustOrientationTaskPid->error(oriModelError);
  ctl.adjustOrientationTaskPid->errorI(ctl.oriErrorI);
  ctl.adjustOrientationTaskPid->errorD(oriModelVel);

  /* Decrease orientation task weight */
  double curOriWeight = ctl.moveContactTask->orientationTaskSp->weight();
  double tarOriWeight = 0;
  ctl.moveContactTask->orientationTaskSp->weight(std::max(curOriWeight - 0.01, tarOriWeight));
  /* Increase adjust orientation task weight */
  double curAdjOriWeight = ctl.adjustOrientationTaskPid->weight();
  double tarAdjOriWeight = contactConf.contactTask.orientation.finalWeight;
  ctl.adjustOrientationTaskPid->weight(std::min(curAdjOriWeight + 1, tarAdjOriWeight));
  /* Increase adjust position task weight */
  double curAdjWeight = ctl.adjustPositionTaskPid->weight();
  double tarAdjWeight = contactConf.contactTask.position.weight;
  ctl.adjustPositionTaskPid->weight(std::min(curAdjWeight + 1, tarAdjWeight));
  /* Decrease position task sp weight */
  double curPWeight = ctl.moveContactTask->positionTaskSp->weight();
  double tarPWeight = 0;
  ctl.moveContactTask->positionTaskSp->weight(std::max(curPWeight - 1, tarPWeight));

  /* Compute objective */
  mc_rbdyn::StanceConfig::ContactObj & obj = contactConf.contactObj;
  sva::MotionVecd robotSurfaceVel = ctl.moveContactTask->robotSurfaceVel();
  double oriErr = oriModelError.norm();
  double posErr = error.norm();

  double velErr = robotSurfaceVel.linear().norm();

  sva::MotionVecd robotBodyVel = ctl.robot().mbc->bodyVelW[bodyIndex];
  if(robotBodyVel.linear().norm() > 0.3)
  {
    ctl.halted = true;
    std::cerr << robotSurfaceVel.linear().norm() << std::endl;
    std::cerr << "OOPS Drive too fast" << std::endl;
  }

  bool insert = posErr < obj.adjustPosThresh and oriErr < obj.adjustOriThresh and velErr < obj.adjustVelThresh;
  if(insert)
  {
    ctl.qpsolver->solver.removeTask(ctl.adjustPositionTaskPid.get());
    ctl.qpsolver->solver.removeTask(ctl.adjustOrientationTaskPid.get());
    ctl.moveContactTask->removeFromSolver(ctl.qpsolver->solver);
    ctl.removeMetaTask(ctl.moveContactTask.get());
    ctl.moveContactTask.reset();

    std::cout << "Finished adjust gripper T" << std::endl;
    return true;
  }

  return false;
}

bool enter_addGripperT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  std::cout << "addGripperT" << std::endl;
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();

  ctl.addContactTask.reset(new mc_tasks::AddContactTask(ctl.robots(), ctl.constSpeedConstr, *(ctl.targetContact), contactConf, 0));
  ctl.addContactTask->addToSolver(ctl.qpsolver->solver);
  ctl.metaTasks.push_back(ctl.addContactTask.get());

  bool leftHand = ctl.targetContact->r1Surface->name() == "LeftGripper";
  if(leftHand)
  {
    ctl.lowerPGainsJoints = {"LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6"};
  }
  else
  {
    ctl.lowerPGainsJoints = {"RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6"};
  }
  ctl.lowerPGainsOriginalValues.clear();
  for(const auto & jn : ctl.lowerPGainsJoints)
  {
    double pgain = 0;
    if(pdgains::getPGain(jn, pgain))
    {
      std::cout << "Original pgain at " << jn << ": " << pgain << std::endl;
      ctl.lowerPGainsOriginalValues.push_back(pgain);
      if(pdgains::setPGain(jn, pgain/10))
      {
        std::cout << "Set pgain at " << jn << " to " << pgain/10 << std::endl;
      }
    }
    else
    {
      std::cout << "Failed to get original pgain value for " << jn << std::endl;
      ctl.lowerPGainsOriginalValues.push_back(-1);
    }
  }

  return true;
}

bool live_addGripperT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  bool ok = ctl.inContact(ctl.addContactTask->robotSurf->name());

  if(ok) /* Python compares nrIterNoContact with np.inf/timeStep (so np.inf) */
  {
    std::cout << "Contact detected" << std::endl;
    ctl.addContactTask->removeFromSolver(ctl.qpsolver->solver);
    ctl.removeMetaTask(ctl.addContactTask.get());
    ctl.addContactTask.reset();
    return true;
  }

  return false;
}

bool enter_removeBeforeCloseT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  bool limitToZero = ctl.targetContact->r2Surface->name() == "PlatformLeftRampVS" or ctl.targetContact->r2Surface->name() == "PlatformLeftRampS"; /*FIXME Should be part of the configuration */
  if(!limitToZero)
  {
    std::cout << "Moving the gripper away before grasp" << std::endl;
    mc_rbdyn::StanceConfig & contactConf = ctl.targetConf();

    /* Find the contact to remove */
    mc_rbdyn::Contact & removedContact = *(ctl.targetContact);

    /* Create the remove contact meta task */
    ctl.removeContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, removedContact, contactConf));
    ctl.removeContactTask->addToSolver(ctl.qpsolver->solver);
    ctl.metaTasks.push_back(ctl.removeContactTask.get());

    /* Get the current position of the wrist */
    ctl.contactPos = ctl.robot().mbc->bodyPosW[ctl.robot().bodyIndexByName(ctl.targetContact->r1Surface->bodyName())].translation();
  }
  return true;
}

bool live_removeBeforeCloseT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  bool limitToZero = ctl.targetContact->r2Surface->name() == "PlatformLeftRampVS" or ctl.targetContact->r2Surface->name() == "PlatformLeftRampS"; /*FIXME Should be part of the configuration */
  if(!limitToZero)
  {
    Eigen::Vector3d curPos = ctl.robot().mbc->bodyPosW[ctl.robot().bodyIndexByName(ctl.targetContact->r1Surface->bodyName())].translation();
    double d = (curPos - ctl.contactPos).norm();
    if(d > 0.005)
    {
      ctl.removeContactTask->removeFromSolver(ctl.qpsolver->solver);
      ctl.removeMetaTask(ctl.removeContactTask.get());
      ctl.removeContactTask.reset();
      ctl.distPairs.clear();
      std::cout << "Moved away, will now grasp" << std::endl;
      return true;
    }
    return false;
  }
  else
  {
    return true;
  }
}

bool enter_softCloseGripperP::eval(MCSeqController & ctl)
{
  std::cout << "enter_softCloseGripperP" << std::endl;
  ctl.isGripperClose = false;

  std::shared_ptr<mc_rbdyn::Surface> robotSurf = ctl.targetContact->r1Surface;
  unsigned int bodyId = ctl.robot().bodyIdByName(robotSurf->bodyName());
  Eigen::MatrixXd dofMat = Eigen::MatrixXd::Zero(6,6);
  for(size_t i = 0; i < 6; ++i) { dofMat(i,i) = 1; }
  Eigen::VectorXd speedMat = Eigen::VectorXd::Zero(6);
  ctl.constSpeedConstr->addBoundedSpeed(ctl.robots().mbs, bodyId, robotSurf->X_b_s().translation(), dofMat, speedMat);
  ctl.qpsolver->solver.updateConstrsNrVars(ctl.robots().mbs);
  ctl.qpsolver->solver.updateConstrSize();

  return true;
}

bool live_softCloseGripperP::eval(MCSeqController & ctl)
{
  bool finish = false;

  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen -= 0.0005;
    bool limitToZero = ctl.targetContact->r2Surface->name() == "PlatformLeftRampVS" or ctl.targetContact->r2Surface->name() == "PlatformLeftRampS"; /*FIXME Should be part of the configuration */
    double percentOpenLimit = limitToZero ? 0.35 : 0.1;
    if(ctl.currentGripper->overCommandLimit || ctl.currentGripper->percentOpen <= percentOpenLimit)
    {
      if(!ctl.currentGripper->overCommandLimit)
      {
        ctl.currentGripper->percentOpen = percentOpenLimit;
      }
      finish = true;
    }
  }
  else
  {
    finish = true;
  }

  if(finish)
  {
    ctl.isGripperClose = true;
    std::shared_ptr<mc_rbdyn::Surface> robotSurf = ctl.targetContact->r1Surface;
    unsigned int bodyId = ctl.robot().bodyIdByName(robotSurf->bodyName());
    ctl.constSpeedConstr->removeBoundedSpeed(bodyId);
    ctl.qpsolver->solver.updateConstrsNrVars(ctl.robots().mbs);
    ctl.qpsolver->solver.updateConstrSize();
    std::cout << "Finished softCloseGripperP" << std::endl;
    return true;
  }

  return false;
}

bool enter_hardCloseGripperP::eval(MCSeqController & ctl)
{
  std::cout << "enter_hardCloseGripperP" << std::endl;
  ctl.isGripperClose = false;

  ctl.iterSinceHardClose = 0;
  return true;
}

bool live_hardCloseGripperP::eval(MCSeqController & ctl)
{
  bool finish = false;

  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen -= 0.0005;
    bool limitToZero = ctl.targetContact->r2Surface->name() == "PlatformLeftRampVS" or ctl.targetContact->r2Surface->name() == "PlatformLeftRampS"; /*FIXME Should be part of the configuration */
    double percentOpenLimit = limitToZero ? 0.25 : 0;
    if(ctl.currentGripper->overCommandLimit || ctl.currentGripper->percentOpen <= percentOpenLimit)
    {
      if(!ctl.currentGripper->overCommandLimit)
      {
        ctl.currentGripper->percentOpen = percentOpenLimit;
      }
      ctl.currentGripper = 0;
      std::cout << "Gripper closed, waiting for adjustment" << std::endl;
    }
  }
  else
  {
    /* We enter here once the gripper has been fully closed, wait a little for the hand to adjust itself */
    //ctl.iterSinceHardClose++;
    //if(ctl.iterSinceHardClose > 3/ctl.timeStep) /*Wait 3 seconds */
    //{
    finish = true;
    //}
  }

  if(finish)
  {
    ctl.isGripperClose = true;
    if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
    /*TODO Get the actual position of the hand, set it in the QP and reset the gains to their original value*/
    bool leftHand = ctl.targetContact->r1Surface->name() == "LeftGripper";
    unsigned int ji = leftHand ? 24 : 16;
    std::vector<double> & eValues = ctl.encoderValues;
    for(const auto & jn : ctl.lowerPGainsJoints)
    {
      ctl.robot().mbc->q[ctl.robot().jointIndexByName(jn)][0] = eValues[ji];
      ji++;
    }
    /* At this point, the robot mbc holds the true values for the arm that was left loose */
    ctl.stabilityTask->postureTask->posture(ctl.robot().mbc->q);
    rbd::forwardKinematics(*(ctl.robot().mb), *(ctl.robot().mbc));
    rbd::forwardVelocity(*(ctl.robot().mb), *(ctl.robot().mbc));
    ctl.stabilityTask->comObj = rbd::computeCoM(*(ctl.robot().mb), *(ctl.robot().mbc));
    ctl.stabilityTask->comTaskSm.reset(ctl.curConf().comTask.weight, ctl.stabilityTask->comObj, ctl.curConf().comTask.targetSpeed);
    std::cout << "Finished hardCloseGripperP" << std::endl;
    return true;
  }

  return false;
}

bool enter_restoreArmGainsP::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  std::cout << "Restoring arm gains to their original values" << std::endl;
  for(size_t i = 0; i < ctl.lowerPGainsJoints.size(); ++i)
  {
    if(ctl.lowerPGainsOriginalValues[i] >= 0)
    {
      if(not pdgains::setPGain(ctl.lowerPGainsJoints[i], ctl.lowerPGainsOriginalValues[i]))
      {
        std::cout << "Failed to restore gain for joint " << ctl.lowerPGainsJoints[i] << std::endl;
      }
    }
  }
  return true;
}

bool enter_contactGripperP::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved) { return true; }
  std::cout << "enter_contactGripperP" << std::endl;
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();

  ctl.removeContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, *(ctl.targetContact), contactConf));
  ctl.removeContactTask->addToSolver(ctl.qpsolver->solver);
  ctl.metaTasks.push_back(ctl.removeContactTask.get());

  return true;
}

bool live_contactGripperT::eval(MCSeqController & ctl)
{
  if((not ctl.isGripperWillBeAttached) and ctl.isRemoved)
  {
    mc_rbdyn::Stance & newS = ctl.targetStance();

    ctl.updateRobotEnvCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateSelfCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateContacts(newS.contacts());
    ctl.updateSolverEqInEq();

    ctl.stanceIndex += 1;
    return true;
  }
  bool ok = true; /*FIXME Python used to check that _back was in the sensorContacts */

  if(ok)
  {
    std::cout << "ok contactGripperT" << std::endl;
    ctl.removeContactTask->removeFromSolver(ctl.qpsolver->solver);
    ctl.removeMetaTask(ctl.removeContactTask.get());
    ctl.removeContactTask.reset();

    mc_rbdyn::Stance & newS = ctl.targetStance();

    ctl.updateRobotEnvCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateSelfCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateContacts(newS.contacts());
    ctl.updateSolverEqInEq();

    ctl.stanceIndex += 1;
    return true;
  }
  else
  {
    return false;
  }
}

}
