#include <mc_control/mc_seq_controller.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <Tasks/QPContactConstr.h>

#include <mc_control/SimulationContactSensor.h>
#include <mc_control/ForceContactSensor.h>

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
  collsConstraint(robots(), timeStep), currentContact(0), targetContact(0), currentGripper(0),
  use_real_sensors(false)
{
  /* Load plan */
  loadStances(seq_path, stances, actions);
  assert(stances.size() == actions.size());
  /*FIXME Load configs from a file */
  configs.resize(stances.size());
  for(size_t i = 0; i < stances.size(); ++i)
  {
    seq_actions.push_back(seqActionFromStanceAction(stances[i], *(actions[i].get())));
  }

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
}

bool MCSeqController::run()
{
  bool ret = true;
  if(!halted && !paused && stanceIndex < seq_actions.size())
  {
    ret = MCController::run();
    if(ret)
    {
      sensorContacts = contactSensor->update(*this);
      pre_live(); /* pre_live can halt the execution */
      if(!halted && seq_actions[stanceIndex]->execute(*this))
      {
        /*FIXME Should pause here to mimic ask_step */
        stanceIndex++;
      }
      post_live();
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
      std::shared_ptr<tasks::qp::GripperTorqueTask> gTask(new tasks::qp::GripperTorqueTask(contactId, robSurf.X_b_motor.translation(), T, 1e-4));
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
      std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp(new tasks::qp::SetPointTask(robots().mbs, 0, positionTask.get(), 20, 10000.));
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

std::vector<std::string> MCSeqController::bodiesFromContacts(const mc_rbdyn::Robot & robot, const std::vector<mc_rbdyn::Contact> & robotContacts)
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

  res.push_back(contact.surfaces());

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
  return std::find(sensorContacts.begin(), sensorContacts.end(), currentContact->robotSurface->name) != sensorContacts.end();
}

void MCSeqController::removeMetaTask(mc_tasks::MetaTask* mt)
{
  metaTasks.erase(std::find(metaTasks.begin(), metaTasks.end(), mt));
}

std::shared_ptr<SeqAction> seqActionFromStanceAction(mc_rbdyn::Stance & stance, mc_rbdyn::StanceAction & action)
{
  /*FIXME Implement stuff*/
  auto res = std::shared_ptr<SeqAction>(new SeqAction());
  res->steps.push_back(SeqStep());
  return res;
}

SeqAction::SeqAction()
: currentStep(0), steps(0)
{
}

bool SeqAction::execute(MCSeqController & controller)
{
  if(steps[currentStep].eval(controller))
  {
    currentStep++;
    if(currentStep == steps.size())
    {
      return true;
    }
  }
  return false;
}

bool SeqStep::eval(MCSeqController & controller)
{
  return false;
}

}
