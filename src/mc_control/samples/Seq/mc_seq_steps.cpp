#include "mc_seq_steps.h"
#include "MCSeqPublisher.h"

#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include "pdgains.cpp"

#include <mc_rbdyn/Surface.h>

namespace mc_control
{

bool enter_initT::eval(MCSeqController &)
{
  LOG_INFO("INIT")
  return true;
}

bool live_initT::eval(MCSeqController & controller)
{
  if(controller.stanceIndex != 0)
  {
    return true;
  }
  controller.updateRobotEnvCollisions(controller.stances[controller.stanceIndex].contacts(), controller.configs[controller.stanceIndex]);
  controller.updateSelfCollisions(controller.stances[controller.stanceIndex].contacts(), controller.configs[controller.stanceIndex]);
  controller.updateContacts(controller.stances[controller.stanceIndex].contacts());

  if(controller.stabilityTask->postureTask->eval().norm() < 0.5)
  {
    LOG_INFO("INIT done")
    controller.stanceIndex += 1;
    return true;
  }
  return false;
}

bool live_chooseContactT::eval(MCSeqController & ctl)
{
  /* Will be stuck here if the contact is not supported */
  mc_rbdyn::StanceAction * curAction = &(ctl.curAction());
  mc_rbdyn::StanceAction * targetAction = &(ctl.targetAction());
  mc_rbdyn::RemoveContactAction * rmCurAction = dynamic_cast<mc_rbdyn::RemoveContactAction*>(curAction);
  mc_rbdyn::AddContactAction * addTargetAction = dynamic_cast<mc_rbdyn::AddContactAction*>(targetAction);
  bool isAddRemove = rmCurAction != 0 && addTargetAction != 0 &&
                     rmCurAction->contact().r1Surface()->bodyName() == addTargetAction->contact().r1Surface()->bodyName();
  if(isAddRemove && rmCurAction->contact().r1Surface()->type() != "gripper")
  {
    ctl.currentContact = &(rmCurAction->contact());
    ctl.targetContact  = &(addTargetAction->contact());
    return true;
  }
  bool isAddOnly = addTargetAction != 0;
  if(isAddOnly && addTargetAction->contact().r1Surface()->type() != "gripper")
  {
    ctl.currentContact = 0;
    ctl.targetContact = &(addTargetAction->contact());
    std::string bodyName = ctl.targetContact->r1Surface()->bodyName();
    /* FIXME Hard-coded */
    if(bodyName == "RARM_LINK6")
    {
      ctl.currentGripper = ctl.grippers["r_gripper"].get();
    }
    else if(bodyName == "LARM_LINK6")
    {
      ctl.currentGripper = ctl.grippers["l_gripper"].get();
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
  LOG_INFO("MOVECONTACT")
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
  ctl.solver().addTask(ctl.rmContactTask);
  ctl.metaTasks.push_back(ctl.rmContactTask.get());

  ctl.updateRobotEnvCollisions(curS.stabContacts(), contactConf);
  ctl.updateSelfCollisions(curS.stabContacts(), contactConf);
  ctl.updateContacts(curS.stabContacts());

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

  bool notInContact = ! ctl.inContact(ctl.currentContact->r1Surface()->name());
  if(notInContact)
  {
    ctl.notInContactCount++;
  }

  if(ctl.notInContactCount > 50)
  {
    ctl.solver().removeTask(ctl.rmContactTask);
    ctl.removeMetaTask(ctl.rmContactTask.get());
    ctl.rmContactTask.reset();

    if(ctl.isColl)
    {
      ctl.updateRobotEnvCollisions(ctl.curStance().stabContacts(), ctl.curConf());
    }

    return true;
  }

  return false;
}

bool enter_moveWPT::eval(MCSeqController & ctl)
{
  ctl.startPolygonInterpolator = true;
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  mc_rbdyn::Stance & newS = ctl.targetStance();

  /* Configure the stability task */
  ctl.stabilityTask->target(ctl.env(), newS, contactConf, contactConf.comTask.targetSpeed);

  /* Create and setup move contact task */
  ctl.moveContactTask.reset(new mc_tasks::MoveContactTask(ctl.robots(), ctl.robot(), ctl.env(), *ctl.targetContact, contactConf, 0.5));
  ctl.solver().addTask(ctl.moveContactTask);
  ctl.metaTasks.push_back(ctl.moveContactTask.get());

  ctl.moveContactTask->toWaypoint(contactConf, contactConf.contactTask.position.targetSpeed);

  return true;
}

bool live_moveWPT::eval(MCSeqController & ctl)
{
  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen[0] += 0.001;
    if(ctl.currentGripper->percentOpen[0] >= 1)
    {
      ctl.currentGripper->percentOpen[0] = 1;
      ctl.currentGripper = 0;
    }
  }
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();

  double curOriWeight = ctl.moveContactTask->orientationTaskSp->weight();
  double tarOriWeight = contactConf.contactTask.orientation.finalWeight;
  ctl.moveContactTask->orientationTaskSp->weight(std::min(curOriWeight + 0.5, tarOriWeight));

  Eigen::Vector3d robotSurfacePos = ctl.moveContactTask->robotSurfacePos().translation();
  Eigen::Vector3d targetPos = ctl.moveContactTask->wp;
  ctl.publisher->publish_waypoint(sva::PTransformd(targetPos));
  double error = (robotSurfacePos - targetPos).norm();

  if(contactConf.contactTask.waypointConf.skip || error < contactConf.contactTask.waypointConf.thresh)
  {
    ctl.moveContactTask->toPreEnv(contactConf, contactConf.contactTask.position.targetSpeed);
    if(ctl.currentGripper == 0)
    {
      ctl.publisher->publish_waypoint(sva::PTransformd::Identity());
      LOG_INFO("Finished to move to contact wp")
      return true;
    }
    else if(ctl.currentGripper->percentOpen[0] >= 1)
    {
      LOG_INFO("Finished to move to contact wp")
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
  //LOG_INFO("Current stance com")
  //LOG_INFO(curSCoM)
  //LOG_INFO("Next stance com")
  //Eigen::Vector3d tarSCoM = ctl.stances[ctl.stanceIndex+1].com(ctl.robot());
  //LOG_INFO(tarSCoM)
  //contactConf.comObj.comOffset = (tarSCoM - curSCoM);
  //contactConf.comObj.comOffset(0) /= 3;
  //contactConf.comObj.comOffset(1) /= 2;
  //contactConf.comObj.comOffset(2) /= 3;
  //LOG_INFO("Offset")
  //LOG_INFO(contactConf.comObj.comOffset)
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  contactConf.comObj.comOffset = contactConf.comObj.comAdjustOffset;
  mc_rbdyn::Stance & newS = ctl.targetStance();
  ctl.stabilityTask->target(ctl.env(), newS, contactConf, contactConf.comTask.targetSpeed);

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

  sva::PTransformd X_slam_target = ctl.publisher->get_slam_contact();
  if(X_slam_target != sva::PTransformd::Identity())
  {
    ctl.moveContactTask->set_target_tf(X_slam_target, ctl.curConf());
    ctl.moveContactTask->toPreEnv(ctl.curConf(), ctl.curConf().contactTask.position.targetSpeed);
  }

  /* Remove collision avoidance when velocity reaches 0 to set the contact more precisely */
  if( (!ctl.isCollFiltered) && velErr < obj.velThresh )
  {
    ctl.setCollisionsContactFilter(*(ctl.targetContact), ctl.curConf());
    ctl.isCollFiltered = true;
  }

  bool inContact = ctl.inContact(ctl.targetContact->r1Surface()->name());
  if( (posErr < obj.posThresh && velErr < obj.velThresh) || inContact )
  {
    ctl.solver().removeTask(ctl.moveContactTask);
    ctl.removeMetaTask(ctl.moveContactTask.get());
    ctl.moveContactTask.reset();

    if(!ctl.isCollFiltered)
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
  bool inContact = ctl.inContact(ctl.targetContact->r1Surface()->name());

  /* If not already in contact, move the contact in the normal direction */
  if(!inContact)
  {
    auto & contactConf = ctl.curConf();
    ctl.addContactTask.reset(new mc_tasks::AddContactTask(ctl.robots(), ctl.constSpeedConstr, *(ctl.targetContact), contactConf));

    std::shared_ptr<mc_rbdyn::Surface> robotSurf = ctl.targetContact->r1Surface();
    bool useComplianceTask = contactConf.contactObj.useComplianceTask && (!ctl.is_simulation);
    if(useComplianceTask)
    {
      LOG_INFO("Using compliance task with target torque: " << contactConf.contactObj.complianceTargetTorque.transpose()
                                    << " and target force: " << contactConf.contactObj.complianceTargetForce.transpose()
                                    << " compliance vel thresh: " << contactConf.contactObj.complianceVelThresh)

      ctl.complianceTask = std::shared_ptr<mc_tasks::ComplianceTask>(new mc_tasks::ComplianceTask(ctl.robots(),
          ctl.robots().robotIndex(), robotSurf->bodyName(), ctl.timeStep, 10.0, 100000.));

      ctl.complianceTask->setTargetWrench(sva::ForceVecd(contactConf.contactObj.complianceTargetTorque, contactConf.contactObj.complianceTargetForce));

      ctl.solver().addTask(ctl.complianceTask);
      ctl.metaTasks.push_back(ctl.complianceTask.get());
    }
    else
    {
      ctl.solver().addTask(ctl.addContactTask);
      ctl.metaTasks.push_back(ctl.addContactTask.get());
    }
    ctl.push = true;
  }

  return true;
}

bool live_pushContactT::eval(MCSeqController & ctl)
{
  bool inContact = ctl.inContact(ctl.targetContact->r1Surface()->name());

  auto & contactConf = ctl.curConf();
  bool useComplianceTask = contactConf.contactObj.useComplianceTask && (!ctl.is_simulation);
  double complianceTargetF = contactConf.contactObj.complianceTargetForce.norm();
  if((!useComplianceTask && inContact) || (useComplianceTask && ctl.complianceTask->speed().norm() < contactConf.contactObj.complianceVelThresh && ctl.complianceTask->eval().norm() < complianceTargetF/2))
  {
    /* Remove AddContactTask if we had it, else remove complianceTask */
    if(ctl.push)
    {
      if(useComplianceTask)
      {
        ctl.solver().removeTask(ctl.complianceTask);
        ctl.removeMetaTask(ctl.complianceTask.get());
      }
      else
      {
        ctl.solver().removeTask(ctl.addContactTask);
        ctl.removeMetaTask(ctl.addContactTask.get());
        ctl.addContactTask.reset();
      }
    }
    ctl.currentContact = 0;
    ctl.targetContact = 0;

    mc_rbdyn::Stance & newS = ctl.targetStance();

    ctl.updateRobotEnvCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateSelfCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateContacts(newS.contacts());

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
    std::string bodyName = rmCurAction->contact().r1Surface()->bodyName();
    if(bodyName == "RARM_LINK6") /*FIXME hard-coded */
    {
      ctl.currentGripper = ctl.grippers["r_gripper"].get();
      ctl.currentGripperIsClosed = ctl.currentGripper->percentOpen[0] < 0.5;
    }
    else if(bodyName == "LARM_LINK6")
    {
      ctl.currentGripper = ctl.grippers["l_gripper"].get();
      ctl.currentGripperIsClosed = ctl.currentGripper->percentOpen[0] < 0.5;
    }
  }
  mc_rbdyn::StanceAction * targetAction = &(ctl.targetAction());
  if(targetAction->type() == "remove")
  {
    LOG_INFO("This action is a removal")
    mc_rbdyn::RemoveContactAction * rmCurAction = dynamic_cast<mc_rbdyn::RemoveContactAction*>(targetAction);
    std::string bodyName = rmCurAction->contact().r1Surface()->bodyName();
    if(bodyName == "RARM_LINK6") /*FIXME hard-coded */
    {
      ctl.currentGripper = ctl.grippers["r_gripper"].get();
      ctl.currentGripperIsClosed = ctl.currentGripper->percentOpen[0] < 0.5;
    }
    else if(bodyName == "LARM_LINK6")
    {
      ctl.currentGripper = ctl.grippers["l_gripper"].get();
      ctl.currentGripperIsClosed = ctl.currentGripper->percentOpen[0] < 0.5;
    }
    if((bodyName == "RARM_LINK6" || bodyName == "LARM_LINK6") && ctl.actions[ctl.stanceIndex+1]->type() != "add")
    {
      LOG_INFO("Will move gripper away from contact in CoM")
      ctl.comRemoveGripper = true;
      ctl.currentContact = &(rmCurAction->contact());
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
  LOG_INFO("COMP")
  ctl.startPolygonInterpolator = true;
  ctl.stabilityTask->target(ctl.env(), ctl.targetStance(), ctl.curConf(), ctl.curConf().comTask.targetSpeed);
  ctl.notInContactCount = 0;
  /*FIXME Should be optionnal */
  //ctl.stabilityTask->highStiffness({"RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6"});
  return true;
}

bool live_CoMOpenGripperT::eval(MCSeqController & ctl)
{
  if(ctl.currentGripper && ctl.currentGripperIsClosed && ctl.comRemoveGripper)
  {
    ctl.currentGripper->percentOpen[0] += 0.001;
    if(ctl.currentGripper->percentOpen[0] >= 1)
    {
      ctl.currentGripper->percentOpen[0] = 1;
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
    LOG_INFO("CoMRemoveGripperT")
    mc_rbdyn::StanceConfig & contactConf = ctl.targetConf();

    /* Find the contact to remove */
    mc_rbdyn::Contact & removedContact = *(ctl.currentContact);

    /* Create the remove contact meta task */
    ctl.removeContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, removedContact, contactConf));
    ctl.solver().addTask(ctl.removeContactTask);
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
    ctl.updateRobotEnvCollisions(targetS.stabContacts(), contactConf);
    ctl.updateSelfCollisions(targetS.stabContacts(), contactConf);
    ctl.updateContacts(targetS.stabContacts());

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
      ctl.solver().removeTask(ctl.removeContactTask);
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

  double error = (ctl.stabilityTask->comObj - rbd::computeCoM(ctl.robot().mb(), ctl.robot().mbc())).norm();
  double errorVel = rbd::computeCoMVelocity(ctl.robot().mb(), ctl.robot().mbc()).norm();

  if( (error < obj.posThresh && errorVel < obj.velThresh) || ctl.notInContactCount > obj.timeout*1/ctl.timeStep)
  {
    if(ctl.stanceIndex + 1 == ctl.stances.size())
    {
      double alphaNorm = 0;
      for(const auto & q : ctl.robot().mbc().alpha)
      {
        for(const auto & qi : q)
        {
          alphaNorm += qi*qi;
        }
      }
      if(alphaNorm > 0.001)
      {
        return false;
      }
    }
    if(ctl.notInContactCount > obj.timeout*1/ctl.timeStep)
    {
      LOG_WARNING("COMP timeout (" << obj.timeout << "s)")
    }
    ctl.notInContactCount = 0;
    ctl.stabilityTask->comObj = rbd::computeCoM(ctl.robot().mb(), ctl.robot().mbc());
    ctl.stabilityTask->comTaskSm.reset(10, ctl.stabilityTask->comObj, 1);
    ctl.stabilityTask->postureTask->weight(100);
    ctl.stabilityTask->postureTask->posture(ctl.robot().mbc().q);

    mc_rbdyn::Stance & newS = ctl.targetStance();
    ctl.updateRobotEnvCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateSelfCollisions(newS.contacts(), ctl.targetConf());
    //ctl.updateContacts(newS.stabContacts());

    //ctl.stabilityTask->normalStiffness({"RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6"});

    return true;
  }

  ctl.notInContactCount++;
  return false;
}

bool live_CoMCloseGripperT::eval(MCSeqController & ctl)
{
  //if(ctl.currentGripper)
  //{
  //  ctl.currentGripper->percentOpen[0] -= 0.0005;
  //  if(ctl.currentGripper->overCommandLimit[0] || ctl.currentGripper->percentOpen[0] <= 0)
  //  {
  //    if(!ctl.currentGripper->overCommandLimit[0])
  //    {
  //      ctl.currentGripper->percentOpen[0] = 0;
  //    }
  //    ctl.currentGripper = 0;
  //    ctl.stanceIndex++;
  //    return true;
  //  }
  //  return false;
  //}
  //else
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
  bool isRemoveAdd = (curRm != 0) && (tarAdd != 0);
  if(isRemoveAdd)
  {
    isRemoveAdd = isRemoveAdd && (curRm->contact().r1Surface()->bodyName() == tarAdd->contact().r1Surface()->bodyName());
  }
  bool isAddOnly = tarAdd != 0;
  bool isRemoveOnly = tarRm != 0;
  if( (isRemoveAdd || isAddOnly) && tarAdd->contact().r1Surface()->type() == "gripper" )
  {
    if(isRemoveAdd)
    {
      ctl.isGripperAttached = true;
      ctl.isGripperWillBeAttached = true;
      ctl.currentContact = &(curRm->contact());
      ctl.targetContact =  &(tarAdd->contact());
    }
    else
    {
      ctl.isGripperAttached = false;
      ctl.isGripperWillBeAttached = true;
      ctl.currentContact = &(tarAdd->contact());
      ctl.targetContact =  &(tarAdd->contact());
    }
    return true;
  }
  if(isRemoveOnly && tarRm->contact().r1Surface()->type() == "gripper")
  {
    ctl.isGripperAttached = true;
    ctl.isGripperWillBeAttached = false;
    ctl.currentContact = &(tarRm->contact());
    ctl.targetContact =  &(tarRm->contact());
    return true;
  }

  std::cout << "\rStuck in chooseGripperT, cannot handle the situation" << std::flush;
  return false;
}

bool enter_openGripperP::eval(MCSeqController & ctl)
{
  mc_rbdyn::Surface & robSurf = *(ctl.currentContact->r1Surface().get());
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
    ctl.currentGripper = ctl.grippers["r_gripper"].get();
  }
  else if(bodyName == "LARM_LINK6")
  {
    ctl.currentGripper = ctl.grippers["l_gripper"].get();
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
    ctl.currentGripper->percentOpen[0] += 0.001;
    if(ctl.currentGripper->percentOpen[0] >= 1.)
    {
      ctl.currentGripper->percentOpen[0] = 1.;
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
  return ctl.isGripperAttached && ctl.isGripperOpen;
}

bool live_openGripperNotRmT::eval(MCSeqController & ctl)
{
  return (!ctl.isGripperAttached) && ctl.isGripperOpen;
}

bool enter_removeGripperP::eval(MCSeqController & ctl)
{
  ctl.skipRemoveGripper = (!ctl.isGripperAttached) && ctl.isGripperOpen;
  if(ctl.skipRemoveGripper)
  {
    return true;
  }
  LOG_INFO("RemoveGripperP")
  mc_rbdyn::StanceConfig & contactConf = (ctl.currentContact == ctl.targetContact) ? ctl.targetConf() : ctl.curConf();

  /* Find the contact to remove */
  mc_rbdyn::Contact & removedContact = *(ctl.currentContact);

  /* Create the remove contact meta task */
  ctl.removeContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, removedContact, contactConf));
  ctl.solver().addTask(ctl.removeContactTask);
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
  /* Get the current position of the wrist */
  ctl.contactPos = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName(removedContact.r1Surface()->bodyName())].translation();

  /* Configure the stability task */
  ctl.stabilityTask->target(ctl.env(), curS, contactConf, contactConf.comTask.targetSpeed);

  /* Configure the QP */
  ctl.updateRobotEnvCollisions(curS.stabContacts(), contactConf);
  ctl.updateSelfCollisions(curS.stabContacts(), contactConf);
  ctl.updateContacts(curS.stabContacts());

  /* Remove collision avoidance between env and moving body */
  ctl.isColl = ctl.setCollisionsContactFilter(*(ctl.currentContact), contactConf);

  ctl.isRemoved = false;
  ctl.notInContactCount = 0;

  return true;
}

bool live_removeGripperP::eval(MCSeqController & ctl)
{
  if(ctl.skipRemoveGripper)
  {
    return true;
  }
  double timeout = 10.*1/ctl.timeStep; //FIXME Should be part of the configuration
  bool all = true;
  double dOut = 0.025;
  double minD = 1;
  for(const auto & p : ctl.distPairs)
  {
    double d = p->distance(ctl.robot(), ctl.env());
    minD = std::min(minD, d);
    all = all && (d > dOut*dOut);
  }
  Eigen::Vector3d curPos = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName(ctl.currentContact->r1Surface()->bodyName())].translation();
  double d = (curPos - ctl.contactPos).norm();
  /* Either we moved away from the model or we moved away "enough" */
  if(all)
  {
    LOG_INFO("Will move away because got out of the model collision")
  }
  double dLimit = 0.15;
  if(ctl.stanceIndex > 18 && ctl.stanceIndex < 41)
  {
    dLimit = 0.05;
  }
  if(ctl.stanceIndex > 40)
  {
    dLimit = 0.15;
  }
  if(minD < 0 && d > dLimit)
  {
    LOG_INFO("Will move away because d > " << dLimit << " even if in model collision")
  }
  if(ctl.notInContactCount++ > timeout)
  {
    LOG_WARNING("Will move away because of timeout")
  }
  all = all || (minD < 0 && d > dLimit) || ctl.notInContactCount > timeout;
  if(all)
  {
    ctl.solver().removeTask(ctl.removeContactTask);
    ctl.removeMetaTask(ctl.removeContactTask.get());
    ctl.removeContactTask.reset();
    ctl.distPairs.clear();

    if(ctl.isColl)
    {
      ctl.updateRobotEnvCollisions(ctl.curStance().stabContacts(), ctl.curConf());
    }
    ctl.isRemoved = true;
    return true;
  }

  return false;
}

bool live_removeGripperT::eval(MCSeqController & ctl)
{
  return ctl.isGripperWillBeAttached && ctl.isRemoved;
}

bool live_removeGripperNotAddT::eval(MCSeqController & ctl)
{
  return (!ctl.isGripperWillBeAttached) && ctl.isRemoved;
}

bool enter_moveGripperWPT::eval(MCSeqController & ctl)
{
  ctl.startPolygonInterpolator = true;
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  LOG_INFO("Move gripper WPT")
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  mc_rbdyn::Stance & newS = ctl.targetStance();

  /* Create and setup move contact task */
  ctl.moveContactTask.reset(new mc_tasks::MoveContactTask(ctl.robots(), ctl.robot(), ctl.env(), *(ctl.targetContact), contactConf));
  ctl.solver().addTask(ctl.moveContactTask);
  ctl.metaTasks.push_back(ctl.moveContactTask.get());
  ctl.moveContactTask->toWaypoint(contactConf, contactConf.contactTask.position.targetSpeed);

  /* Configure the stability task */
  ctl.stabilityTask->target(ctl.env(), newS, contactConf, contactConf.comTask.targetSpeed);

  ctl.isBodyTask = false;
  if(ctl.curStance().contacts().size() <= 2)
  {
    unsigned int bodyIndex = ctl.robot().bodyIndexByName("BODY");
    ctl.bodyOriTask.reset(new tasks::qp::OrientationTask(ctl.robots().mbs(), 0, "BODY", ctl.robot().mbc().bodyPosW[bodyIndex].rotation()));
    ctl.bodyOriTaskSp.reset(new tasks::qp::SetPointTask(ctl.robots().mbs(), 0, ctl.bodyOriTask.get(), 10, 1000));
    ctl.solver().addTask(ctl.bodyOriTaskSp.get());
    ctl.isBodyTask = true;
  }

  return true;
}

bool live_moveGripperWPT::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();

  Eigen::Vector3d robotSurfacePos = ctl.moveContactTask->robotSurfacePos().translation();
  Eigen::Vector3d targetPos = ctl.moveContactTask->wp;
  ctl.publisher->publish_waypoint(sva::PTransformd(targetPos));
  double error = (robotSurfacePos - targetPos).norm();
  if(contactConf.contactTask.waypointConf.skip || error < contactConf.contactTask.waypointConf.thresh)
  {
    /* Waypoint reached, new goal is target */
    ctl.moveContactTask->toPreEnv(contactConf, contactConf.contactTask.position.targetSpeed);
    ctl.publisher->publish_waypoint(sva::PTransformd::Identity());
    ctl.setCollisionsContactFilter(*(ctl.targetContact), ctl.curConf());
    LOG_INFO("Finished move gripper WPT")
    return true;
  }
  else
  {
    return false;
  }
}

bool live_moveGripperT::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  mc_rbdyn::StanceConfig::ContactObj & obj = ctl.curConf().contactObj;

  Eigen::Vector3d robotSurfacePos = ctl.moveContactTask->robotSurfacePos().translation();
  Eigen::Vector3d targetPos = ctl.moveContactTask->preTargetPos;
  double posErr = (robotSurfacePos - targetPos).norm();
  double velErr = ctl.moveContactTask->robotSurfaceVel().linear().norm();

  sva::PTransformd X_slam_target = ctl.publisher->get_slam_contact();
  if(X_slam_target != sva::PTransformd::Identity())
  {
    ctl.moveContactTask->set_target_tf(X_slam_target, ctl.curConf());
    ctl.moveContactTask->toPreEnv(ctl.curConf(), ctl.curConf().contactTask.position.targetSpeed);
  }

  if(ctl.isBodyTask)
  {
    double curBodyOriWeight = ctl.bodyOriTaskSp->weight();
    ctl.bodyOriTaskSp->weight(std::max(0.0, curBodyOriWeight - 0.5));
  }

  if(posErr < obj.posThresh && velErr < obj.velThresh)
  {
    if(ctl.isBodyTask)
    {
      ctl.solver().removeTask(ctl.bodyOriTaskSp.get());
    }
    LOG_INFO("Finished move gripper T")
    return true;
  }

  return false;
}

bool enter_adjustGripperP::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  LOG_INFO("Adjust Gripper P")
  double stiff = ctl.moveContactTask->posStiff + ctl.moveContactTask->extraPosStiff;
  ctl.adjustPositionTask.reset(new tasks::qp::PositionTask(ctl.robots().mbs(), 0, ctl.moveContactTask->robotSurf->bodyName(),
                                                          ctl.moveContactTask->positionTask->position(),
                                                          ctl.moveContactTask->robotSurf->X_b_s().translation()));
  ctl.adjustOrientationTask.reset(new tasks::qp::OrientationTask(ctl.robots().mbs(), 0, ctl.moveContactTask->robotSurf->bodyName(), ctl.moveContactTask->targetOri));
  ctl.adjustPositionTaskPid.reset(new tasks::qp::PIDTask(ctl.robots().mbs(), 0, ctl.adjustPositionTask.get(), stiff, 4, 2*std::sqrt(stiff), 0));
  double oriStiff = ctl.moveContactTask->orientationTaskSp->stiffness();
  ctl.adjustOrientationTaskPid.reset(new tasks::qp::PIDTask(ctl.robots().mbs(), 0, ctl.adjustOrientationTask.get(), oriStiff, 0.5, 2*std::sqrt(oriStiff), 0));

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
  ctl.solver().addTask(ctl.adjustPositionTaskPid.get());

  Eigen::Vector3d oriModelError = sva::rotationError(ctl.robot().mbc().bodyPosW[ctl.moveContactTask->robotBodyIndex].rotation(),
                                                     ctl.moveContactTask->targetOri, 1e-7);
  Eigen::Vector3d oriModelVel = M_0_s.angular();
  ctl.adjustOrientationTaskPid->error(oriModelError);
  ctl.adjustOrientationTaskPid->errorI(Eigen::Vector3d(0,0,0));
  ctl.adjustOrientationTaskPid->errorD(oriModelVel);
  ctl.oriErrorI = Eigen::Vector3d(0,0,0);
  ctl.solver().addTask(ctl.adjustOrientationTaskPid.get());

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
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();
  unsigned int bodyIndex = ctl.moveContactTask->robotBodyIndex;

  sva::PTransformd X_0_b = ctl.robot().mbc().bodyPosW[bodyIndex];
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

  sva::MotionVecd robotBodyVel = ctl.robot().mbc().bodyVelW[bodyIndex];
  if(robotBodyVel.linear().norm() > 0.3)
  {
    ctl.halted = true;
    LOG_ERROR(robotSurfaceVel.linear().norm())
    LOG_ERROR("OOPS Drive too fast")
  }

  bool insert = posErr < obj.adjustPosThresh && oriErr < obj.adjustOriThresh && velErr < obj.adjustVelThresh;
  if(insert)
  {
    ctl.solver().removeTask(ctl.adjustPositionTaskPid.get());
    ctl.solver().removeTask(ctl.adjustOrientationTaskPid.get());
    ctl.solver().removeTask(ctl.moveContactTask);
    ctl.removeMetaTask(ctl.moveContactTask.get());
    ctl.moveContactTask.reset();

    LOG_INFO("Finished adjust gripper T")
    return true;
  }

  return false;
}

bool enter_addGripperT::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  LOG_INFO("addGripperT")
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();

  ctl.notInContactCount = 0;
  ctl.addContactTask.reset(new mc_tasks::AddContactTask(ctl.robots(), ctl.constSpeedConstr, *(ctl.targetContact), contactConf, 0));
  ctl.solver().addTask(ctl.addContactTask);
  ctl.metaTasks.push_back(ctl.addContactTask.get());

  //bool leftHand = ctl.targetContact->r1Surface()->name() == "LeftGripper";
  //if(leftHand)
  //{
  //  ctl.lowerPGainsJoints = {"LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6"};
  //}
  //else
  //{
  //  ctl.lowerPGainsJoints = {"RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6"};
  //}
  //ctl.lowerPGainsOriginalValues.clear();
  //for(const auto & jn : ctl.lowerPGainsJoints)
  //{
  //  double pgain = 0;
  //  if(pdgains::getPGain(jn, pgain))
  //  {
  //    LOG_INFO("Original pgain at " << jn << ": " << pgain)
  //    ctl.lowerPGainsOriginalValues.push_back(pgain);
  //    if(pdgains::setPGain(jn, pgain/10))
  //    {
  //      LOG_INFO("Set pgain at " << jn << " to " << pgain/10)
  //    }
  //  }
  //  else
  //  {
  //    LOG_INFO("Failed to get original pgain value for " << jn)
  //    ctl.lowerPGainsOriginalValues.push_back(-1);
  //  }
  //}

  return true;
}

bool live_addGripperT::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  bool ok = ctl.inContact(ctl.addContactTask->robotSurf->name());

  if(ok) /* Python compares nrIterNoContact with np.inf/timeStep (so np.inf) */
  {
    LOG_INFO("Contact detected")
    ctl.notInContactCount = 0;
    ctl.solver().removeTask(ctl.addContactTask);
    ctl.removeMetaTask(ctl.addContactTask.get());
    ctl.addContactTask.reset();
    return true;
  }
  ctl.notInContactCount++;
  ///* Should be configurable per stance */
  //if(ctl.notInContactCount > 8*1/ctl.timeStep)
  //{
  //  LOG_WARNING("No contact detected since 8 seconds (" << ctl.notInContactCount << " iterations), skip ahead")
  //  ctl.solver().removeTask(ctl.addContactTask);
  //  ctl.removeMetaTask(ctl.addContactTask.get());
  //  ctl.addContactTask.reset();
  //  return true;
  //}

  return false;
}

bool enter_removeBeforeCloseT::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  if(ctl.notInContactCount > 0) { return true; }
  mc_rbdyn::StanceConfig & contactConf = ctl.targetConf();
  LOG_INFO("Moving the gripper away before grasp (Move " << ctl.curConf().contactObj.gripperMoveAwayDist*100 << " cm)")

  /* Find the contact to remove */
  mc_rbdyn::Contact & removedContact = *(ctl.targetContact);

  /* Create the remove contact meta task */
  ctl.removeContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, removedContact, contactConf));
  ctl.solver().addTask(ctl.removeContactTask);
  ctl.metaTasks.push_back(ctl.removeContactTask.get());

  /* Get the current position of the wrist */
  ctl.contactPos = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName(ctl.targetContact->r1Surface()->bodyName())].translation();
  return true;
}

bool live_removeBeforeCloseT::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  if(ctl.notInContactCount > 0) { ctl.notInContactCount = 0; return true; }
  Eigen::Vector3d curPos = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName(ctl.targetContact->r1Surface()->bodyName())].translation();
  double d = (curPos - ctl.contactPos).norm();
  if(d > ctl.curConf().contactObj.gripperMoveAwayDist)
  {
    ctl.solver().removeTask(ctl.removeContactTask);
    ctl.removeMetaTask(ctl.removeContactTask.get());
    ctl.removeContactTask.reset();
    ctl.distPairs.clear();
    LOG_INFO("Moved away, will now grasp")
    return true;
  }
  return false;
}

bool enter_softCloseGripperP::eval(MCSeqController & ctl)
{
  LOG_INFO("enter_softCloseGripperP")
  ctl.isGripperClose = false;

  std::shared_ptr<mc_rbdyn::Surface> robotSurf = ctl.targetContact->r1Surface();
  ctl.complianceTask = std::shared_ptr<mc_tasks::ComplianceTask>(new mc_tasks::ComplianceTask(ctl.robots(),
      ctl.robots().robotIndex(), robotSurf->bodyName(), ctl.timeStep, 10.0, 100000., 3., 1., {0.005, 0}, {0.05, 0}));
  if(!ctl.is_simulation)
  {
    ctl.solver().addTask(ctl.complianceTask);
    ctl.metaTasks.push_back(ctl.complianceTask.get());
    ctl.constSpeedConstr->removeBoundedSpeed(ctl.solver(), robotSurf->bodyName());
  }
  else
  {
    Eigen::MatrixXd dofMat = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd speedMat = Eigen::VectorXd::Zero(6);
    ctl.constSpeedConstr->addBoundedSpeed(ctl.solver(), robotSurf->bodyName(), robotSurf->X_b_s().translation(), dofMat, speedMat);
  }
  return true;
}

bool live_softCloseGripperP::eval(MCSeqController & ctl)
{
  bool finish = false;

  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen[0] -= 0.0005;
    //bool limitToZero = ctl.targetContact->r2Surface()->name() == "PlatformLeftRampVS" or ctl.targetContact->r2Surface()->name() == "PlatformLeftRampS"; /*FIXME Should be part of the configuration */
    double percentOpenLimit = 0.;//limitToZero ? 0.35 : 0.1;
    if(ctl.currentGripper->overCommandLimit[0] || ctl.currentGripper->percentOpen[0] <= percentOpenLimit)
    {
      if(!ctl.currentGripper->overCommandLimit[0])
      {
        ctl.currentGripper->percentOpen[0] = percentOpenLimit;
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
    if(!ctl.is_simulation)
    {
      ctl.solver().removeTask(ctl.complianceTask);
      ctl.removeMetaTask(ctl.complianceTask.get());
    }
    else
    {
      std::shared_ptr<mc_rbdyn::Surface> robotSurf = ctl.targetContact->r1Surface();
      ctl.constSpeedConstr->removeBoundedSpeed(ctl.solver(), robotSurf->bodyName());
    }
    LOG_INFO("Finished softCloseGripperP")
    return true;
  }

  return false;
}

bool enter_hardCloseGripperP::eval(MCSeqController & ctl)
{
  LOG_INFO("enter_hardCloseGripperP")
  ctl.isGripperClose = false;

  ctl.iterSinceHardClose = 0;
  return true;
}

bool live_hardCloseGripperP::eval(MCSeqController & ctl)
{
  bool finish = false;

  if(ctl.currentGripper)
  {
    ctl.currentGripper->percentOpen[0] -= 0.0005;
    bool limitToZero = ctl.targetContact->r2Surface()->name() == "PlatformLeftRampVS" || ctl.targetContact->r2Surface()->name() == "PlatformLeftRampS"; /*FIXME Should be part of the configuration */
    double percentOpenLimit = limitToZero ? 0.25 : 0;
    if(ctl.currentGripper->overCommandLimit[0] || ctl.currentGripper->percentOpen[0] <= percentOpenLimit)
    {
      if(!ctl.currentGripper->overCommandLimit[0])
      {
        ctl.currentGripper->percentOpen[0] = percentOpenLimit;
      }
      ctl.currentGripper = 0;
      LOG_INFO("Gripper closed, waiting for adjustment")
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
    if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
    /*TODO Get the actual position of the hand, set it in the QP and reset the gains to their original value*/
    //bool leftHand = ctl.targetContact->r1Surface()->name() == "LeftGripper";
    //unsigned int ji = leftHand ? 24 : 16;
    //std::vector<double> & eValues = ctl.encoderValues;
    //for(const auto & jn : ctl.lowerPGainsJoints)
    //{
    //  ctl.robot().mbc().q[ctl.robot().jointIndexByName(jn)][0] = eValues[ji];
    //  ji++;
    //}
    ///* At this point, the robot mbc holds the true values for the arm that was left loose */
    //ctl.stabilityTask->postureTask->posture(ctl.robot().mbc().q);
    //rbd::forwardKinematics(*(ctl.robot().mb), *(ctl.robot().mbc));
    //rbd::forwardVelocity(*(ctl.robot().mb), *(ctl.robot().mbc));
    //ctl.stabilityTask->comObj = rbd::computeCoM(*(ctl.robot().mb), *(ctl.robot().mbc));
    //ctl.stabilityTask->comTaskSm.reset(ctl.curConf().comTask.weight, ctl.stabilityTask->comObj, ctl.curConf().comTask.targetSpeed);
    LOG_INFO("Finished hardCloseGripperP")
    return true;
  }

  return false;
}

bool enter_restoreArmGainsP::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  //LOG_INFO("Restoring arm gains to their original values")
  //for(size_t i = 0; i < ctl.lowerPGainsJoints.size(); ++i)
  //{
  //  if(ctl.lowerPGainsOriginalValues[i] >= 0)
  //  {
  //    if(not pdgains::setPGain(ctl.lowerPGainsJoints[i], ctl.lowerPGainsOriginalValues[i]))
  //    {
  //      LOG_INFO("Failed to restore gain for joint " << ctl.lowerPGainsJoints[i])
  //    }
  //  }
  //}
  return true;
}

bool enter_contactGripperP::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved) { return true; }
  LOG_INFO("enter_contactGripperP")
  mc_rbdyn::StanceConfig & contactConf = ctl.curConf();

  ctl.removeContactTask.reset(new mc_tasks::RemoveContactTask(ctl.robots(), ctl.constSpeedConstr, *(ctl.targetContact), contactConf));
  ctl.solver().addTask(ctl.removeContactTask);
  ctl.metaTasks.push_back(ctl.removeContactTask.get());

  return true;
}

bool live_contactGripperT::eval(MCSeqController & ctl)
{
  if((!ctl.isGripperWillBeAttached) && ctl.isRemoved)
  {
    ctl.stanceIndex += 1;
    mc_rbdyn::Stance & newS = ctl.targetStance();

    ctl.updateRobotEnvCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateSelfCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateContacts(newS.contacts());
    return true;
  }
  bool ok = true; /*FIXME Python used to check that _back was in the sensorContacts */

  if(ok)
  {
    LOG_INFO("ok contactGripperT")
    ctl.solver().removeTask(ctl.removeContactTask);
    ctl.removeMetaTask(ctl.removeContactTask.get());
    ctl.removeContactTask.reset();

    mc_rbdyn::Stance & newS = ctl.targetStance();

    ctl.updateRobotEnvCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateSelfCollisions(newS.contacts(), ctl.targetConf());
    ctl.updateContacts(newS.contacts());

    ctl.stanceIndex += 1;
    return true;
  }
  else
  {
    return false;
  }
}

}
