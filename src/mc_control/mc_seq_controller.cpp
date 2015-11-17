#include <mc_control/mc_seq_controller.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <Tasks/QPContactConstr.h>

#include <mc_control/SimulationContactSensor.h>
#include <mc_control/ForceContactSensor.h>

#include <mc_control/mc_seq_steps.h>

#include <mc_rbdyn/GripperSurface.h>

#include <mc_rbdyn/json/StanceConfig.h>

#include <mc_rtc/logging.h>

#include <fstream>

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
  const auto & hull1 = r1.convex(r1bodyName);
  X_b1_h1 = r1.collisionTransform(hull1.first);
  r1hull = hull1.second;
  const auto & hull2 = r2.convex(r2bodyName);
  X_b2_h2 = r2.collisionTransform(hull2.first);
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
  sch::transform(*(r1hull.get()), X_b1_h1*r1.mbc().bodyPosW[r1BodyIndex]);
  sch::transform(*(r2hull.get()), X_b2_h2*r2.mbc().bodyPosW[r2BodyIndex]);
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

MCSeqTimeLog::MCSeqTimeLog(double timeStep)
: timeStep(timeStep), phases(0), iters(0)
{
}

void MCSeqTimeLog::logPhase(const std::string & phase, uint64_t iter)
{
  phases.push_back(phase);
  iters.push_back(iter);
}

void MCSeqTimeLog::report()
{
  report(std::cout);
}

void MCSeqTimeLog::report(const std::string & file)
{
  std::ofstream ofs(file);
  report(ofs);
}

void MCSeqTimeLog::report(std::ostream & os)
{
  for(size_t i = 1; i < phases.size(); ++i)
  {
    os << "Phase " << phases[i] << ": " << (static_cast<double>(iters[i] - iters[i-1]))*timeStep << "s" << std::endl;
  }
  os << "Total time: " << (static_cast<double>(iters.back() - iters[0]))*timeStep << std::endl;
}

MCSeqController::MCSeqController(const std::string & env_name, const std::string & seq_path, bool real_sensors, unsigned int start_stance, bool step_by_step)
: MCSeqController(mc_rtc::MC_ENV_DESCRIPTION_PATH, env_name, seq_path, real_sensors, start_stance, step_by_step)
{
}

MCSeqController::MCSeqController(const std::string & env_path, const std::string & env_name, const std::string & seq_path, bool real_sensors, unsigned int start_stance, bool step_by_step)
: MCSeqController(std::make_shared<mc_robots::EnvRobotModule>(env_path, env_name), seq_path, real_sensors, start_stance, step_by_step)
{
}

MCSeqController::MCSeqController(const std::shared_ptr<mc_rbdyn::RobotModule> & env_module, const std::string & seq_path, bool real_sensors, unsigned int start_stance, bool step_by_step)
: MCController(env_module),
  nrIter(0), logger(timeStep),
  step_by_step(step_by_step), paused(false), halted(false),
  stanceIndex(start_stance), seq_actions(0),
  currentContact(0), targetContact(0), currentGripper(0),
  use_real_sensors(real_sensors),
  collsConstraint(robots(), timeStep)
{
  logger.logPhase("START", 0);
  /* Load plan */
  loadStances(robots(), seq_path, stances, actions);
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
  /* Load plan configuration */
  std::string config_path = seq_path;
  config_path.replace(config_path.find(".json"), strlen(".json"), "_config.json");
  loadStanceConfigs(config_path);
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
  auto q = robot().mbc().q;
  auto ff = stances[stanceIndex].q()[0];
  q[0] = {ff[0], ff[1], ff[2], ff[3], ff[4], ff[5], ff[6]};
  robot().mbc().q = q;
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  constSpeedConstr.reset(new tasks::qp::BoundedSpeedConstr(robots().mbs(), 0, timeStep));
  constSpeedConstr->addToSolver(qpsolver->solver);

  dynamicsConstraint = mc_solver::DynamicsConstraint(qpsolver->robots, 0, timeStep,
                                                         false, {0.01, 0.001, 1e-6}, 0.5);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(collsConstraint);
  qpsolver->setContacts(stances[stanceIndex].geomContacts());

  qpsolver->update();

  stabilityTask.reset(new mc_tasks::StabilityTask(robots()));
  stabilityTask->addToSolver(qpsolver->solver);
  metaTasks.push_back(stabilityTask.get());
  stabilityTask->target(env(), stances[stanceIndex], configs[stanceIndex], configs[stanceIndex].comTask.targetSpeed);

  LOG_SUCCESS("MCSeqController init done")
  LOG_INFO("Setup to play " << seq_actions.size() << " actions")
}

bool MCSeqController::run()
{
  bool ret = true;
  if(!halted && !paused && stanceIndex < seq_actions.size())
  {
    ret = MCController::run();
    nrIter++;
    if(ret)
    {
      unsigned int stanceIndexIn = stanceIndex;
      sensorContacts = contactSensor->update(*this);
      pre_live(); /* pre_live can halt the execution */
      if(!halted && seq_actions[stanceIndex]->execute(*this))
      {
        if(stanceIndex != stanceIndexIn)
        {
          LOG_SUCCESS("Completed " << actions[stanceIndexIn]->toStr())
          logger.logPhase(actions[stanceIndexIn]->toStr(), nrIter);
          logger.report();
          logger.report("/tmp/mc-control-seq-times.log");
          if(stanceIndex < actions.size())
          {
            LOG_INFO("Starting " << actions[stanceIndex]->toStr() << "(" << (stanceIndex+1) << "/" << actions.size() << ")")
            /*FIXME Disabled for now... */
            //Eigen::Vector3d pos(robot().mbc().q[0][4], robot().mbc().q[0][5], robot().mbc().q[0][6]);
            //std::vector<double> rPose = robot().mbc().q[0];
            //robot().mbc().zero(robot().mb());
            //std::vector<double> & eValues = encoderValues;
            //for(size_t i = 0; i < 24; ++i)
            //{
            //  robot().mbc().q[i+1][0] = eValues[i];
            //}
            //for(size_t i = 24; i < 32; ++i)
            //{
            //  robot().mbc().q[i+6][0] = eValues[i];
            //}
            ///* At this point, the robot mbc holds the encoder values */
            //rbd::forwardKinematics(robot().mb(), robot().mbc());
            //Eigen::Vector3d & rpy = sensorOri;
            //sva::PTransformd chestSensorOri(sva::RotZ(rpy(2))*sva::RotY(rpy(1))*sva::RotX(rpy(0)), Eigen::Vector3d(0,0,0));
            //sva::PTransformd X_0_body = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
            //sva::PTransformd X_0_acce = robot().mbc().bodyPosW[robot().bodyIndexByName("CHEST_LINK1")];
            //sva::PTransformd X_acce_body = X_0_body * X_0_acce.inv();
            //X_0_acce = sva::PTransformd(chestSensorOri.rotation(), X_0_acce.translation());
            //sva::PTransformd bodySensorOri = X_acce_body * X_0_acce;
            //Eigen::Quaterniond q(bodySensorOri.rotation());
            //q.normalize();
            //q = q.inverse();
            //robot().mbc().q[0][0] = q.w();
            //robot().mbc().q[0][1] = q.x();
            //robot().mbc().q[0][2] = q.y();
            //robot().mbc().q[0][3] = q.z();
            //robot().mbc().q[0][4] = pos(0);
            //robot().mbc().q[0][5] = pos(1);
            //robot().mbc().q[0][6] = pos(2);
            //robot().mbc().q[0] = rPose;
            //rbd::forwardKinematics(robot().mb(), robot().mbc());
            //qpsolver->setContacts(stances[stanceIndex-1].geomContacts);
            ////qpsolver->update();
            //stabilityTask->postureTask->posture(robot().mbc().q);
            //stabilityTask->comObj = stances[stanceIndex-1].com(robot());//rbd::computeCoM(robot().mb(), robot().mbc());
            //stabilityTask->comTaskSm.reset(curConf().comTask.weight, stabilityTask->comObj, curConf().comTask.targetSpeed);
          }
          paused = step_by_step;
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

  /* Reset the free flyer to the free flyer in the sequence */
  robot().mbc().zero(robot().mb());
  robot().mbc().q = stabilityTask->postureTask->posture();
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
  qpsolver->setContacts(stances[stanceIndex].geomContacts());
  LOG_INFO("LFullSole position: " << robot().surface("LFullSole").X_0_s(robot()).translation().transpose())
  LOG_INFO("RFullSole position: " << robot().surface("RFullSole").X_0_s(robot()).translation().transpose())
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
    mc_rbdyn::GripperSurface * is_gs = dynamic_cast<mc_rbdyn::GripperSurface*>(c.r1Surface().get());
    if(is_gs)
    {
      mc_rbdyn::GripperSurface & robSurf = *is_gs;
      tasks::qp::ContactId contactId = c.contactId(robots());
      Eigen::Vector3d T = robSurf.X_b_motor().rotation().row(0);
      std::shared_ptr<tasks::qp::GripperTorqueTask> gTask(new tasks::qp::GripperTorqueTask(contactId, robSurf.X_b_motor().translation(), T, 1e-4));
      gripperTorqueTasks.push_back(gTask);
      qpsolver->solver.addTask(gTask.get());
    }
  }

  for(const auto & c : contacts)
  {
    mc_rbdyn::GripperSurface * is_gs = dynamic_cast<mc_rbdyn::GripperSurface*>(c.r1Surface().get());
    std::string bodyName = c.r1Surface()->bodyName();
    if(is_gs and actiGrippers.count(bodyName) == 0 and robot().hasForceSensor(bodyName) )
    {
      LOG_INFO("ActiGripper ADD " << bodyName)
      std::string forceSensor = robot().forceSensorByBody(bodyName);
      unsigned int wrenchIndex = forceSensor == "RightHandForceSensor" ? 2 : 3; /*FIXME Hard-coded */
      tasks::qp::ContactId contactId = c.contactId(robots());
      sva::PTransformd X_0_s = c.r1Surface()->X_0_s(robot());
      double actiForce = 10; /* FIXME Hard-coded, should at least be an acti gripper const static member */
      double stopForce = 100; /* FIXME ^^ */
      std::shared_ptr<tasks::qp::PositionTask> positionTask(new tasks::qp::PositionTask(robots().mbs(), 0, contactId.r1BodyId, X_0_s.translation(), is_gs->X_b_s().translation()));
      std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp(new tasks::qp::SetPointTask(robots().mbs(), 0, positionTask.get(), 20, 100000.));
      qpsolver->solver.addTask(positionTaskSp.get());
      actiGrippers[bodyName] = ActiGripper(wrenchIndex, actiForce, stopForce, contactId, X_0_s, 0.04, positionTask, positionTaskSp); /*FIXME 0.04 is ActiGripperMaxPull */
    }
  }

  std::vector<std::string> contactBodies;
  std::vector<std::string> rmActi;
  for(const auto & c : contacts)
  {
    contactBodies.push_back(c.r1Surface()->bodyName());
  }
  for(const auto & ka : actiGrippers)
  {
    if(std::find(contactBodies.begin(), contactBodies.end(), ka.first) == contactBodies.end())
    {
      LOG_INFO("ActiGripper RM " << ka.first);
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
            LOG_ERROR("OOPS too much error")
          }
        }
      }
      else
      {
        if(forceNorm < ba.second.actiForce)
        {
          Eigen::MatrixXd dof = Eigen::MatrixXd::Zero(5,6);
          for(int i = 0; i < 5; ++i)
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
    res.push_back(c.r1Surface()->bodyName());
  }
  return res;
}

std::vector< std::pair<std::string, std::string> >
MCSeqController::collisionsContactFilterList(const mc_rbdyn::Contact & contact, const mc_rbdyn::StanceConfig & conf)
{
  std::vector< std::pair<std::string, std::string> > res;

  res.push_back(std::pair<std::string, std::string>(contact.r1Surface()->bodyName(), contact.r2Surface()->bodyName()));

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
    LOG_INFO("Playing " << actions[stanceIndex]->toStr())
    paused = false;
    return true;
  }
  return false;
}

void MCSeqController::loadStanceConfigs(const std::string & file)
{
  LOG_INFO("Loading stance configs from " << file)
  configs.resize(0);
  Json::Value v;
  {
    std::ifstream ifs(file);
    if(ifs.bad())
    {
      LOG_ERROR("Failed to open configuration file: " << file)
    }
    try
    {
      ifs >> v;
    }
    catch(const std::runtime_error & exc)
    {
      LOG_ERROR("Failed to read configuration file")
      LOG_WARNING(exc.what())
    }
  }
  /*
    The JSON file contains two sections:
    - A General section contains configuration information for the full sequence
    - A StepByStep section contains configuration relative to single steps
  */
  mc_rbdyn::StanceConfig comMoveConfig;
  mc_rbdyn::StanceConfig contactMoveConfig;
  mc_rbdyn::StanceConfig gripperMoveConfig;
  if(v.isMember("General"))
  {
    const Json::Value & scv = v["General"];
    if(scv.isMember("CoMMove"))
    {
      mc_rbdyn::StanceConfigFromJSON(comMoveConfig, scv["CoMMove"]);
    }
    if(scv.isMember("ContactMove"))
    {
      mc_rbdyn::StanceConfigFromJSON(contactMoveConfig, scv["ContactMove"]);
    }
    if(scv.isMember("GripperMove"))
    {
      mc_rbdyn::StanceConfigFromJSON(gripperMoveConfig, scv["GripperMove"]);
    }
    if(scv.isMember("Collisions"))
    {
      mc_rbdyn::scCollisionsFromJSON(comMoveConfig.collisions, scv["Collisions"]);
      mc_rbdyn::scCollisionsFromJSON(contactMoveConfig.collisions, scv["Collisions"]);
      mc_rbdyn::scCollisionsFromJSON(gripperMoveConfig.collisions, scv["Collisions"]);
    }
  }
  for(size_t i = 1; i < seq_actions.size(); ++i)
  {
    mc_rbdyn::StanceConfig sc;

    /* Copy the general configuration for this step before finding specific information */
    switch(seq_actions[i]->type())
    {
      case SeqAction::CoMMove:
        sc = comMoveConfig;
        break;
      case SeqAction::ContactMove:
        sc = contactMoveConfig;
        break;
      case SeqAction::GripperMove:
        sc = gripperMoveConfig;
        break;
      default:
        throw("Not happenning");
        break;
    }

    /* Look for a matching state in the JSON file */
    if(v.isMember("StepByStep"))
    {
      std::string type = actions[i]->type();
      std::string r1Surface = "";
      std::string r2Surface = "";
      if(type != "identity")
      {
        r1Surface = actions[i]->contact().r1Surface()->name();
        r2Surface = actions[i]->contact().r2Surface()->name();
      }
      for(const auto & scv : v["StepByStep"])
      {
        if(scv["type"] == type && scv["r1Surface"] == r1Surface && scv["r2Surface"] == r2Surface)
        {
          mc_rbdyn::StanceConfigFromJSON(sc, scv);
        }
      }
    }
    configs.push_back(sc);
  }
  configs.push_back(mc_rbdyn::StanceConfig());
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
  bool curIsRemoveContact = false;
  bool curIsGripperContact = false;
  std::string curSurfaceName = "";
  {
    mc_rbdyn::AddContactAction* addA = dynamic_cast<mc_rbdyn::AddContactAction*>(curAction);
    if(addA)
    {
      curIsGripperContact = addA->contact().r1Surface()->type() == "gripper";
      curSurfaceName = addA->contact().r1Surface()->name();
    }
  }
  {
    mc_rbdyn::RemoveContactAction* rmA = dynamic_cast<mc_rbdyn::RemoveContactAction*>(curAction);
    if(rmA)
    {
      curIsRemoveContact = true;
      curIsGripperContact = rmA->contact().r1Surface()->type() == "gripper";
      curSurfaceName = rmA->contact().r1Surface()->name();
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
      targetIsGripperContact = addA->contact().r1Surface()->type() == "gripper";
      targetSurfaceName = addA->contact().r1Surface()->name();
    }
  }
  {
    mc_rbdyn::RemoveContactAction* rmA = dynamic_cast<mc_rbdyn::RemoveContactAction*>(targetAction);
    if(rmA)
    {
      targetIsRemoveContact = true;
      targetIsGripperContact = rmA->contact().r1Surface()->type() == "gripper";
      targetSurfaceName = rmA->contact().r1Surface()->name();
    }
  }
  bool targetTargetIsAddContact = false;
  {
    mc_rbdyn::AddContactAction * addA = dynamic_cast<mc_rbdyn::AddContactAction*>(targetTargetAction);
    if(addA && targetIsRemoveContact && targetIsGripperContact
       && targetSurfaceName == addA->contact().r1Surface()->name())
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
                  //std::shared_ptr<SeqStep>(new enter_hardCloseGripperP()),
                  //std::shared_ptr<SeqStep>(new live_hardCloseGripperP()),
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
    LOG_ERROR("Could not find a branch for the following action couple:")
    LOG_WARNING("Current: " << curAction->toStr())
    LOG_WARNING("Target: " << targetAction->toStr())
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
