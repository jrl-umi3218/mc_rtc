#include <mc_control/mc_seq_controller.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <Tasks/QPContactConstr.h>

#include <mc_control/SimulationContactSensor.h>
#include <mc_control/ForceContactSensor.h>

#include <mc_control/mc_seq_steps.h>

#include <mc_rbdyn/surface.h>

namespace mc_control
{

ActiGripper::ActiGripper()
{
}

ActiGripper::ActiGripper(unsigned int wrenchIndex, double actiForce, double stopForce,
                         tasks::qp::ContactId contactId, sva::PTransformd & X_0_s, double maxDist,
                         const std::shared_ptr<tasks::qp::PositionTask> & positionTask,
                         const std::shared_ptr<tasks::qp::SetPointTask> & positionTaskSp)
: wrenchIndex(wrenchIndex), actiForce(actiForce), stopForce(stopForce),
  contactId(contactId), X_0_s(X_0_s), maxDist(maxDist),
  positionTask(positionTask), positionTaskSp(positionTaskSp),
  activated(false), zVec(X_0_s.rotation().row(2)),
  toRemove(false), targetError(0)
{
}

CollisionPair::CollisionPair(const mc_rbdyn::Robot & r1, const mc_rbdyn::Robot & r2,
                             const std::string & r1bodyName, const std::string & r2bodyName)
: r1BodyIndex(r1.bodyIndexByName(r1bodyName)),
  r2BodyIndex(r2.bodyIndexByName(r2bodyName))
{
  const auto & hull1 = r1.convex.at(r1bodyName);
  X_b1_h1 = r1.collisionTransforms.at(hull1.first);
  r1hull = hull1.second;
  const auto & hull2 = r2.convex.at(r2bodyName);
  X_b2_h2 = r2.collisionTransforms.at(hull2.first);
  r2hull = hull2.second;
  pair.reset(new sch::CD_Pair(r1hull.get(), r2hull.get()));
  pair->setEpsilon(std::numeric_limits<double>::epsilon());
  pair->setRelativePrecision(1e-6);
}

double CollisionPair::distance(const mc_rbdyn::Robot & r1, const mc_rbdyn::Robot & r2)
{
  setTransform(r1, r2);
  return pair->getDistance();
}

void CollisionPair::setTransform(const mc_rbdyn::Robot & r1, const mc_rbdyn::Robot & r2)
{
  sch::transform(*(r1hull.get()), X_b1_h1*r1.mbc->bodyPosW[r1BodyIndex]);
  sch::transform(*(r2hull.get()), X_b2_h2*r2.mbc->bodyPosW[r2BodyIndex]);
}

std::vector<mc_solver::Collision> confToColl(const std::vector<mc_rbdyn::StanceConfig::BodiesCollisionConf> & conf)
{
  std::vector<mc_solver::Collision> res;
  for(const auto & bcc : conf)
  {
    res.push_back(mc_solver::Collision(bcc.body1, bcc.body2,
                                       bcc.collisionConf.iDist,
                                       bcc.collisionConf.sDist,
                                       bcc.collisionConf.damping));
  }

  return res;
}

MCSeqController::MCSeqController(const std::string & env_path, const std::string & env_name, const std::string & seq_path)
: MCController(env_path, env_name), paused(false), halted(false), stanceIndex(0), seq_actions(0),
  currentContact(0), targetContact(0), currentGripper(0),
  use_real_sensors(false),
  collsConstraint(robots(), timeStep)
{
  /* Load plan */
  loadStances(seq_path, stances, actions);
  assert(stances.size() == actions.size());
  seq_actions.push_back(seqActionFromStanceAction(0, actions[0].get(), 0));
  for(size_t i = 0; i < stances.size() - 1; ++i)
  {
    if(i + 2 < stances.size())
    {
      seq_actions.push_back(seqActionFromStanceAction(actions[i].get(), actions[i+1].get(), actions[i+2].get()));
    }
    else
    {
      seq_actions.push_back(seqActionFromStanceAction(actions[i].get(), actions[i+1].get(), 0));
    }
  }
  /*FIXME Hard-coded for stairs climbing */
  /*FIXME Load configs from a file */
  //configs.resize(stances.size());
  configs.resize(0);
  for(size_t i = 1; i < seq_actions.size(); ++i)
  {
    mc_rbdyn::StanceConfig sc;

    if(seq_actions[i]->type() == SeqAction::CoMMove)
    {
      sc.postureTask.stiffness = 0.1;
      sc.postureTask.weight = 10.0;
      sc.comTask.stiffness = 1.0;
      sc.comTask.extraStiffness = 1.0;
      sc.comTask.weight = 400.0;
      sc.comTask.targetSpeed = 0.001;
      sc.comObj.posThresh = 0.05,
      sc.comObj.velThresh = 0.0001;
      sc.comObj.comOffset = Eigen::Vector3d(0,0,0);
    }
    if(seq_actions[i]->type() == SeqAction::ContactMove)
    {
      sc.postureTask.stiffness = 0.1;
      sc.postureTask.weight = 10.0;
      sc.comTask.stiffness = 3.0;
      sc.comTask.extraStiffness = 6.0;
      sc.comTask.weight = 500.0;
      sc.comTask.targetSpeed = 0.003;
      sc.comObj.comOffset = Eigen::Vector3d(0,0,0);
      sc.contactObj.posThresh = 0.03;
      sc.contactObj.velThresh = 0.005;
      sc.contactObj.preContactDist = 0.02;
      sc.contactTask.position.stiffness = 2.0;
      sc.contactTask.position.extraStiffness = 6.0;
      sc.contactTask.position.weight = 600.0;
      sc.contactTask.position.targetSpeed = 0.001;
      sc.contactTask.orientation.stiffness = 1.0;
      sc.contactTask.orientation.weight = 300.0;
      sc.contactTask.orientation.finalWeight = 1000.0;
      sc.contactTask.linVel.stiffness = 1.0;
      sc.contactTask.linVel.weight = 10000.0;
      sc.contactTask.linVel.speed = 0.02;
      sc.contactTask.waypointConf.thresh = 0.15;
      sc.contactTask.waypointConf.pos = mc_rbdyn::percentWaypoint(0.2, 1, 0.9, 0.2);
      sc.contactTask.collisionConf.iDist = 0.01;
      sc.contactTask.collisionConf.sDist = 0.005;
      sc.contactTask.collisionConf.damping = 0.05;
    }
    if(seq_actions[i]->type() == SeqAction::GripperMove)
    {
      sc.postureTask.stiffness = 0.05;
      sc.postureTask.weight = 10.0;
      sc.comTask.stiffness = 0.5;
      sc.comTask.extraStiffness = 0.5;
      sc.comTask.weight = 500.0;
      sc.comTask.targetSpeed = 0.0005;
      sc.contactObj.posThresh = 0.03;
      sc.contactObj.velThresh = 0.05;
      sc.contactObj.adjustPosThresh = 0.05;
      sc.contactObj.adjustVelThresh = 0.02;
      sc.contactObj.adjustOriThresh = 0.1;
      sc.contactObj.adjustOriTBNWeight = Eigen::Vector3d(1,1,1);
      sc.contactObj.preContactDist = 0.02;
      sc.contactTask.position.stiffness = 0.25;
      sc.contactTask.position.extraStiffness = 1.0;
      sc.contactTask.position.weight = 600.0;
      sc.contactTask.position.targetSpeed = 0.0005;
      sc.contactTask.orientation.stiffness = 0.25;
      sc.contactTask.orientation.weight = 200.0;
      sc.contactTask.orientation.finalWeight = 1000.0;
      sc.contactTask.linVel.stiffness = 0.5;
      sc.contactTask.linVel.weight = 1000.0;
      sc.contactTask.linVel.speed = 0.02;
      sc.contactTask.waypointConf.thresh = 0.1;
      sc.contactTask.waypointConf.pos = mc_rbdyn::percentWaypoint(0.3, 0.8, 0.7, 0.2);
      sc.contactTask.collisionConf.iDist = 0.01;
      sc.contactTask.collisionConf.sDist = 0.005;
      sc.contactTask.collisionConf.damping = 0.05;
    }

    /* Still general configuration */
    sc.collisions.autoc.push_back({"RLEG_LINK2", "LLEG_LINK2", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"RLEG_LINK3", "LLEG_LINK3", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"RLEG_LINK5", "LLEG_LINK5", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"RLEG_LINK5", "LLEG_LINK3", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"LLEG_LINK5", "RLEG_LINK3", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "RLEG_LINK3", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "RLEG_LINK2", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "RLEG_LINK3", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "LLEG_LINK2", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "LLEG_LINK3", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "CHEST_LINK1", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "BODY", {0.1, 0.05, 0.0}});
    sc.collisions.robotEnv.push_back({"RARM_LINK6", "stair_step2", {0.1, 0.05, 0.0}});
    sc.collisions.robotEnv.push_back({"RARM_LINK6", "stair_step3", {0.1, 0.05, 0.0}});
    sc.collisions.robotEnv.push_back({"RARM_LINK6", "platform", {0.1, 0.05, 0.0}});
    if(i < 10)
    {
      //sc.collisions.robotEnv.push_back({"RARM_LINK6", "stair_step2", {0.05, 0.01, 0.0}});
      //sc.collisions.robotEnv.push_back({"RARM_LINK6", "stair_step3", {0.05, 0.01, 0.0}});
      //sc.collisions.robotEnv.push_back({"RARM_LINK6", "platform", {0.05, 0.01, 0.0}});
      //sc.collisions.robotEnv.push_back({"RARM_LINK5", "platform", {0.05, 0.01, 0.0}});
      //sc.collisions.robotEnv.push_back({"RARM_LINK4", "platform", {0.05, 0.01, 0.0}});
      //sc.collisions.robotEnv.push_back({"RARM_LINK3", "platform", {0.05, 0.01, 0.0}});
    }

    /* Per-stance configuration */
    mc_rbdyn::AddContactAction* addA = dynamic_cast<mc_rbdyn::AddContactAction*>(actions[i].get());
    //mc_rbdyn::RemoveContactAction* rmA = dynamic_cast<mc_rbdyn::RemoveContactAction*>(actions[i].get());
    if(addA)
    {
      if(addA->contact.robotSurface->name == "LeftGripper" &&
         addA->contact.envSurface->name == "StairLeftRung1")
      {
        sc.comObj.comOffset = Eigen::Vector3d(0.05, 0.0,0.0);
      }
      if(addA->contact.robotSurface->name == "LFrontSole" &&
         addA->contact.envSurface->name == "StairStep1")
      {
      }
    }
    mc_rbdyn::RemoveContactAction* rmA = dynamic_cast<mc_rbdyn::RemoveContactAction*>(actions[i].get());
    if(rmA)
    {
      if(rmA->contact.robotSurface->name == "LFullSole" &&
         rmA->contact.envSurface->name == "Ground")
      {
      }
    }

    configs.push_back(sc);
  }
  configs.push_back(mc_rbdyn::StanceConfig());

  /* Setup contact sensor */
  if(use_real_sensors)
  {
    contactSensor.reset(new ForceContactSensor(robot()));
  }
  else
  {
    contactSensor.reset(new SimulationContactSensor(stances));
  }

  /* Setup the QP */
  auto q = robot().mbc->q;
  auto ff = stances[stanceIndex].q[0];
  q[0] = {ff[0], ff[1], ff[2], ff[3], ff[4], ff[5], ff[6]};
  robot().mbc->q = q;
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  constSpeedConstr.reset(new tasks::qp::BoundedSpeedConstr(robots().mbs, 0, timeStep));
  constSpeedConstr->addToSolver(qpsolver->solver);

  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(collsConstraint);
  qpsolver->setContacts(stances[stanceIndex].geomContacts);

  qpsolver->update();

  stabilityTask.reset(new mc_tasks::StabilityTask(robots()));
  stabilityTask->addToSolver(qpsolver->solver);
  metaTasks.push_back(stabilityTask.get());
  stabilityTask->target(env(), stances[stanceIndex], configs[stanceIndex], configs[stanceIndex].comTask.targetSpeed);

  std::cout << "MCSeqController init done" << std::endl;
  std::cout << "Setup to play " << seq_actions.size() << " actions" << std::endl;
}

bool MCSeqController::run()
{
  bool ret = true;
  if(!halted && !paused && stanceIndex < seq_actions.size())
  {
    ret = MCController::run();
    if(ret)
    {
      unsigned int stanceIndexIn = stanceIndex;
      sensorContacts = contactSensor->update(*this);
      pre_live(); /* pre_live can halt the execution */
      if(!halted && seq_actions[stanceIndex]->execute(*this))
      {
        if(stanceIndex != stanceIndexIn)
        {
          std::cout << "Completed " << actions[stanceIndexIn]->toStr() << std::endl;
          if(stanceIndex < actions.size())
          {
            std::cout << "Starting " << actions[stanceIndex]->toStr() << std::endl;
            /*FIXME Disabled for now... */
            //std::cout << "Before modification by sensor " << robot().mbc->q[0][0] << " " << robot().mbc->q[0][1] << " " << robot().mbc->q[0][2] << " " << robot().mbc->q[0][3] << std::endl;
            //Eigen::Vector3d pos(robot().mbc->q[0][4], robot().mbc->q[0][5], robot().mbc->q[0][6]);
            //std::vector<double> rPose = robot().mbc->q[0];
            //robot().mbc->zero(*(robot().mb));
            //std::vector<double> & eValues = encoderValues;
            //for(size_t i = 0; i < 24; ++i)
            //{
            //  robot().mbc->q[i+1][0] = eValues[i];
            //}
            //for(size_t i = 24; i < 32; ++i)
            //{
            //  robot().mbc->q[i+6][0] = eValues[i];
            //}
            ///* At this point, the robot mbc holds the encoder values */
            //rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
            //Eigen::Vector3d & rpy = sensorOri;
            //sva::PTransformd chestSensorOri(sva::RotZ(rpy(2))*sva::RotY(rpy(1))*sva::RotX(rpy(0)), Eigen::Vector3d(0,0,0));
            //sva::PTransformd X_0_body = robot().mbc->bodyPosW[robot().bodyIndexByName("BODY")];
            //sva::PTransformd X_0_acce = robot().mbc->bodyPosW[robot().bodyIndexByName("CHEST_LINK1")];
            //sva::PTransformd X_acce_body = X_0_body * X_0_acce.inv();
            //X_0_acce = sva::PTransformd(chestSensorOri.rotation(), X_0_acce.translation());
            //sva::PTransformd bodySensorOri = X_acce_body * X_0_acce;
            //Eigen::Quaterniond q(bodySensorOri.rotation());
            //q.normalize();
            //q = q.inverse();
            //robot().mbc->q[0][0] = q.w();
            //robot().mbc->q[0][1] = q.x();
            //robot().mbc->q[0][2] = q.y();
            //robot().mbc->q[0][3] = q.z();
            //robot().mbc->q[0][4] = pos(0);
            //robot().mbc->q[0][5] = pos(1);
            //robot().mbc->q[0][6] = pos(2);
            //robot().mbc->q[0] = rPose;
            //std::cout << "After modification by sensor " << robot().mbc->q[0][0] << " " << robot().mbc->q[0][1] << " " << robot().mbc->q[0][2] << " " << robot().mbc->q[0][3] << std::endl;
            //rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
            //qpsolver->setContacts(stances[stanceIndex-1].geomContacts);
            ////qpsolver->update();
            //stabilityTask->postureTask->posture(robot().mbc->q);
            //stabilityTask->comObj = stances[stanceIndex-1].com(robot());//rbd::computeCoM(*(robot().mb), *(robot().mbc));
            //stabilityTask->comTaskSm.reset(curConf().comTask.weight, stabilityTask->comObj, curConf().comTask.targetSpeed);
          }
          //paused = true;
        }
      }
      post_live();
    }
    else
    {
    }
  }
  return ret;
}

void MCSeqController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  if(reset_data.contacts.size())
  {
    qpsolver->setContacts(reset_data.contacts);
  }
  else
  {
    qpsolver->setContacts(stances[stanceIndex].geomContacts);
  }
  qpsolver->update();
}

void MCSeqController::updateRobotEnvCollisions(const std::vector<mc_rbdyn::Contact> & contacts, const mc_rbdyn::StanceConfig & conf)
{
  collsConstraint.setEnvCollisions(robots(), contacts, confToColl(conf.collisions.robotEnv));
}

void MCSeqController::updateSelfCollisions(const std::vector<mc_rbdyn::Contact> & contacts, const mc_rbdyn::StanceConfig & conf)
{
  collsConstraint.setSelfCollisions(robots(), contacts, confToColl(conf.collisions.autoc));
}

void MCSeqController::updateContacts(const std::vector<mc_rbdyn::Contact> & contacts)
{
  qpsolver->setContacts(contacts);

  for(const auto & gTT : gripperTorqueTasks)
  {
    qpsolver->solver.removeTask(gTT.get());
  }
  gripperTorqueTasks.clear();

  for(const auto & c : contacts)
  {
    mc_rbdyn::GripperSurface * is_gs = dynamic_cast<mc_rbdyn::GripperSurface*>(c.robotSurface.get());
    if(is_gs)
    {
      mc_rbdyn::GripperSurface & robSurf = *is_gs;
      tasks::qp::ContactId contactId = c.contactId(robot(), env());
      Eigen::Vector3d T = robSurf.X_b_motor.rotation().row(0);
      std::shared_ptr<tasks::qp::GripperTorqueTask> gTask(new tasks::qp::GripperTorqueTask(contactId, robSurf.X_b_motor.translation(), T, 10));
      gripperTorqueTasks.push_back(gTask);
      qpsolver->solver.addTask(gTask.get());
    }
  }

  for(const auto & c : contacts)
  {
    mc_rbdyn::GripperSurface * is_gs = dynamic_cast<mc_rbdyn::GripperSurface*>(c.robotSurface.get());
    std::string bodyName = c.robotSurface->bodyName;
    if(is_gs and actiGrippers.count(bodyName) == 0 and robot().hasForceSensor(bodyName) )
    {
      std::cout << "ActiGripper ADD " << bodyName << std::endl;
      std::string forceSensor = robot().forceSensorByBody(bodyName);
      unsigned int wrenchIndex = forceSensor == "RightHandForceSensor" ? 2 : 3; /*FIXME Hard-coded */
      tasks::qp::ContactId contactId = c.contactId(robot(), env());
      sva::PTransformd X_0_s = c.robotSurface->X_0_s(robot());
      double actiForce = 50; /* FIXME Hard-coded, should at least be an acti gripper const static member */
      double stopForce = 90; /* FIXME ^^ */
      std::shared_ptr<tasks::qp::PositionTask> positionTask(new tasks::qp::PositionTask(robots().mbs, 0, contactId.r1BodyId, X_0_s.translation(), is_gs->X_b_s().translation()));
      std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp(new tasks::qp::SetPointTask(robots().mbs, 0, positionTask.get(), 4, 10000.));
      qpsolver->solver.addTask(positionTaskSp.get());
      actiGrippers[bodyName] = ActiGripper(wrenchIndex, actiForce, stopForce, contactId, X_0_s, 0.04, positionTask, positionTaskSp); /*FIXME 0.04 is ActiGripperMaxPull */
    }
  }

  std::vector<std::string> contactBodies;
  std::vector<std::string> rmActi;
  for(const auto & c : contacts)
  {
    contactBodies.push_back(c.robotSurface->bodyName);
  }
  for(const auto & ka : actiGrippers)
  {
    if(std::find(contactBodies.begin(), contactBodies.end(), ka.first) == contactBodies.end())
    {
      std::cout << "ActiGripper RM " << ka.first;
      qpsolver->solver.removeTask(ka.second.positionTaskSp.get());
      /* This cast is guaranted to work */
      (dynamic_cast<tasks::qp::ContactConstr*>(contactConstraint.contactConstr.get()))->removeDofContact(ka.second.contactId);
      rmActi.push_back(ka.first);
    }
  }
  for(const auto & k : rmActi)
  {
    actiGrippers.erase(actiGrippers.find(k));
  }
}

void MCSeqController::updateSolverEqInEq()
{
  qpsolver->update();
}

void MCSeqController::pre_live()
{
  for(auto & ba : actiGrippers)
  {
    const Eigen::Vector3d & force = wrenches[ba.second.wrenchIndex].first;
    double forceNorm = force.norm();
    ba.second.targetError = ba.second.positionTask->eval().norm();
    if(not ba.second.toRemove)
    {
      if(ba.second.activated)
      {
        if(forceNorm > ba.second.stopForce and ba.second.targetError < ba.second.maxDist*0.1)
        {
          std::cout << "DESACTIVATED " << ba.first << std::endl;
          ba.second.activated = false;
          /* This cast is guaranted to work */
          (dynamic_cast<tasks::qp::ContactConstr*>(contactConstraint.contactConstr.get()))->removeDofContact(ba.second.contactId);
          (dynamic_cast<tasks::qp::ContactConstr*>(contactConstraint.contactConstr.get()))->updateDofContacts();
        }
        else
        {
          double err = std::min(std::max(ba.second.actiForce - forceNorm, 0.0)/ba.second.actiForce, 1.0);
          double pos = ba.second.maxDist*err;
          Eigen::Vector3d target = ba.second.X_0_s.translation() + ba.second.zVec*pos;
          ba.second.positionTask->position(target);
          if(ba.second.targetError > ba.second.maxDist*1.5)
          {
            halted = true;
            std::cout << "OOPS too much error" << std::endl;
          }
        }
      }
      else
      {
        if(forceNorm < ba.second.actiForce)
        {
          std::cout << "ACTIVATED " << ba.first << std::endl;
          Eigen::MatrixXd dof(5,6);
          for(size_t i = 0; i < 5; ++i)
          {
            dof(i,i) = 1;
          }
          ba.second.activated = true;
          (dynamic_cast<tasks::qp::ContactConstr*>(contactConstraint.contactConstr.get()))->addDofContact(ba.second.contactId, dof);
          (dynamic_cast<tasks::qp::ContactConstr*>(contactConstraint.contactConstr.get()))->updateDofContacts();
        }
      }
    }
    else
    {
      Eigen::Vector3d target = ba.second.X_0_s.translation();
      ba.second.positionTask->position(target);
    }
  }
}

void MCSeqController::post_live()
{
  for(const auto & t : metaTasks)
  {
    t->update();
  }
}

mc_rbdyn::StanceConfig & MCSeqController::curConf()
{
  return configs[stanceIndex - 1];
}

mc_rbdyn::Stance & MCSeqController::curStance()
{
  return stances[stanceIndex - 1];
}

mc_rbdyn::StanceAction & MCSeqController::curAction()
{
  return *(actions[stanceIndex - 1]);
}

mc_rbdyn::StanceConfig & MCSeqController::targetConf()
{
  return configs[stanceIndex];
}

mc_rbdyn::Stance & MCSeqController::targetStance()
{
  return stances[stanceIndex];
}

mc_rbdyn::StanceAction & MCSeqController::targetAction()
{
  return *(actions[stanceIndex]);
}

std::vector<std::string> MCSeqController::bodiesFromContacts(const mc_rbdyn::Robot &, const std::vector<mc_rbdyn::Contact> & robotContacts)
{
  std::vector<std::string> res;
  for(const auto & c : robotContacts)
  {
    res.push_back(c.robotSurface->bodyName);
  }
  return res;
}

std::vector< std::pair<std::string, std::string> >
MCSeqController::collisionsContactFilterList(const mc_rbdyn::Contact & contact, const mc_rbdyn::StanceConfig & conf)
{
  std::vector< std::pair<std::string, std::string> > res;

  res.push_back(std::pair<std::string, std::string>(contact.robotSurface->bodyName, contact.envSurface->bodyName));

  if(conf.collisions.robotEnvContactFilter.count(contact.surfaces()))
  {
    for(const auto & p : conf.collisions.robotEnvContactFilter.at(contact.surfaces()))
    {
      res.push_back(p);
    }
  }

  return res;
}

bool MCSeqController::setCollisionsContactFilter(const mc_rbdyn::Contact & contact, const mc_rbdyn::StanceConfig & conf)
{
  bool ret = false;
  auto v = collisionsContactFilterList(contact, conf);
  for(const auto & p : v)
  {
    ret |= collsConstraint.removeEnvCollisionByBody(robots(), p.first, p.second);
  }
  return ret;
}

bool MCSeqController::inContact(const std::string & sname)
{
  return std::find(sensorContacts.begin(), sensorContacts.end(), sname) != sensorContacts.end();
}

void MCSeqController::removeMetaTask(mc_tasks::MetaTask* mt)
{
  metaTasks.erase(std::find(metaTasks.begin(), metaTasks.end(), mt));
}

bool MCSeqController::play_next_stance()
{
  if(paused and stanceIndex < stances.size())
  {
    std::cout << "Playing " << actions[stanceIndex]->toStr() << std::endl;
    paused = false;
    return true;
  }
  return false;
}

std::shared_ptr<SeqAction> seqActionFromStanceAction(mc_rbdyn::StanceAction * curAction, mc_rbdyn::StanceAction * targetAction, mc_rbdyn::StanceAction * targetTargetAction)
{
  auto res = std::shared_ptr<SeqAction>(new SeqAction());
  if(curAction == 0)
  {
    res->steps = {
                  std::shared_ptr<SeqStep>(new enter_initT()),
                  std::shared_ptr<SeqStep>(new live_initT())
    };
    res->_type = SeqAction::CoMMove;
    return res;
  }
  bool curIsAddContact = false;
  bool curIsRemoveContact = false;
  bool curIsGripperContact = false;
  std::string curSurfaceName = "";
  {
    mc_rbdyn::AddContactAction* addA = dynamic_cast<mc_rbdyn::AddContactAction*>(curAction);
    if(addA)
    {
      curIsAddContact = true;
      curIsGripperContact = addA->contact.robotSurface->type() == "gripper";
      curSurfaceName = addA->contact.robotSurface->name;
    }
  }
  {
    mc_rbdyn::RemoveContactAction* rmA = dynamic_cast<mc_rbdyn::RemoveContactAction*>(curAction);
    if(rmA)
    {
      curIsRemoveContact = true;
      curIsGripperContact = rmA->contact.robotSurface->type() == "gripper";
      curSurfaceName = rmA->contact.robotSurface->name;
    }
  }
  //bool curIsIdentity = (!curIsAddContact && !curIsRemoveContact);
  bool targetIsAddContact = false;
  bool targetIsRemoveContact = false;
  bool targetIsGripperContact = false;
  std::string targetSurfaceName = "";
  {
    mc_rbdyn::AddContactAction* addA = dynamic_cast<mc_rbdyn::AddContactAction*>(targetAction);
    if(addA)
    {
      targetIsAddContact = true;
      targetIsGripperContact = addA->contact.robotSurface->type() == "gripper";
      targetSurfaceName = addA->contact.robotSurface->name;
    }
  }
  {
    mc_rbdyn::RemoveContactAction* rmA = dynamic_cast<mc_rbdyn::RemoveContactAction*>(targetAction);
    if(rmA)
    {
      targetIsRemoveContact = true;
      targetIsGripperContact = rmA->contact.robotSurface->type() == "gripper";
      targetSurfaceName = rmA->contact.robotSurface->name;
    }
  }
  bool targetTargetIsAddContact = false;
  {
    mc_rbdyn::AddContactAction * addA = dynamic_cast<mc_rbdyn::AddContactAction*>(targetTargetAction);
    if(addA && targetIsRemoveContact && targetIsGripperContact
       && targetSurfaceName == addA->contact.robotSurface->name)
    {
      targetTargetIsAddContact = true;
    }
  }
  bool targetIsIdentity = (!targetIsAddContact && !targetIsRemoveContact);
  bool sameSurface = curSurfaceName == targetSurfaceName;


  bool contactBranch = (curIsRemoveContact && targetIsAddContact && sameSurface  && !curIsGripperContact) || (targetIsAddContact && !targetIsGripperContact);
  bool comBranch = (targetIsRemoveContact or targetIsIdentity);
  bool gripperBranch = (curIsRemoveContact && targetIsAddContact && sameSurface && targetIsGripperContact) || (targetIsAddContact && targetIsGripperContact) || (targetIsRemoveContact && targetIsGripperContact && (not targetTargetIsAddContact));

  if(contactBranch)
  {
    res->steps = {
                  std::shared_ptr<SeqStep>(new live_chooseContactT()),
                  std::shared_ptr<SeqStep>(new enter_removeContactT()),
                  std::shared_ptr<SeqStep>(new live_removeContacT()),
                  std::shared_ptr<SeqStep>(new enter_moveWPT()),
                  std::shared_ptr<SeqStep>(new live_moveWPT()),
                  std::shared_ptr<SeqStep>(new enter_moveContactP()),
                  std::shared_ptr<SeqStep>(new live_moveContactT()),
                  std::shared_ptr<SeqStep>(new enter_pushContactT()),
                  std::shared_ptr<SeqStep>(new live_pushContactT())
    };
    res->_type = SeqAction::ContactMove;
  }
  else if(gripperBranch)
  {
    res->steps = {
                  std::shared_ptr<SeqStep>(new live_chooseGripperT()),
                  std::shared_ptr<SeqStep>(new enter_openGripperP()),
                  std::shared_ptr<SeqStep>(new live_openGripperP()),
                  std::shared_ptr<SeqStep>(new enter_removeGripperP()),
                  std::shared_ptr<SeqStep>(new live_removeGripperP()),
                  std::shared_ptr<SeqStep>(new enter_moveGripperWPT()),
                  std::shared_ptr<SeqStep>(new live_moveGripperWPT()),
                  std::shared_ptr<SeqStep>(new live_moveGripperT()),
                  std::shared_ptr<SeqStep>(new enter_adjustGripperP()),
                  std::shared_ptr<SeqStep>(new live_adjustGripperT()),
                  std::shared_ptr<SeqStep>(new enter_addGripperT()),
                  std::shared_ptr<SeqStep>(new live_addGripperT()),
                  std::shared_ptr<SeqStep>(new enter_removeBeforeCloseT()),
                  std::shared_ptr<SeqStep>(new live_removeBeforeCloseT()),
                  std::shared_ptr<SeqStep>(new enter_softCloseGripperP()),
                  std::shared_ptr<SeqStep>(new live_softCloseGripperP()),
                  std::shared_ptr<SeqStep>(new enter_hardCloseGripperP()),
                  std::shared_ptr<SeqStep>(new live_hardCloseGripperP()),
                  std::shared_ptr<SeqStep>(new enter_restoreArmGainsP()),
                  std::shared_ptr<SeqStep>(new enter_contactGripperP()),
                  std::shared_ptr<SeqStep>(new live_contactGripperT())
    };
    res->_type = SeqAction::GripperMove;
  }
  else if(comBranch)
  {
    res->steps = {
                  std::shared_ptr<SeqStep>(new live_chooseCoMT()),
                  std::shared_ptr<SeqStep>(new enter_moveCoMP()),
                  std::shared_ptr<SeqStep>(new live_moveCoMT())
    };
    res->_type = SeqAction::CoMMove;
  }
  else
  {
    std::cerr << "Could not find a branch for the following action couple:"<< std::endl;
    std::cerr << "Current: " << curAction->toStr() << std::endl;
    std::cerr << "Target: " << targetAction->toStr() << std::endl;
    res->steps = {
                  std::shared_ptr<SeqStep>(new SeqStep())
    };
  }

  return res;
}

SeqAction::SeqAction()
: currentStep(0), steps(0)
{
}

bool SeqAction::execute(MCSeqController & controller)
{
  if(steps[currentStep]->eval(controller))
  {
    currentStep++;
    if(currentStep == steps.size())
    {
      return true;
    }
  }
  return false;
}

SeqAction::SeqActionType SeqAction::type() const
{
  return _type;
}

bool SeqStep::eval(MCSeqController &)
{
  return false;
}

}
