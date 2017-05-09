#include "mc_seq_controller.h"
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <Tasks/QPContactConstr.h>

#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/logging.h>

#include <mc_control/ForceContactSensor.h>
#include <mc_control/SimulationContactSensor.h>

#include "mc_seq_steps.h"
#include "MCSeqPublisher.h"
#include "json/StanceConfig.h"

#include <fstream>

#include <geos/geom/LinearRing.h>
#include <geos/geom/CoordinateSequence.h>

#include "pdgains.cpp"

namespace mc_control
{

ActiGripper::ActiGripper()
{
}

ActiGripper::ActiGripper(const std::string& wrenchName, double actiForce, double stopForce,
                         double actiTorque, double stopTorque,
                         tasks::qp::ContactId contactId, sva::PTransformd & X_0_s,
                         double maxDist, double maxRot,
                         const std::shared_ptr<tasks::qp::PositionTask> & positionTask,
                         const std::shared_ptr<tasks::qp::SetPointTask> & positionTaskSp,
                         const std::shared_ptr<tasks::qp::OrientationTask> & orientationTask,
                         const std::shared_ptr<tasks::qp::SetPointTask> & orientationTaskSp)
: wrenchName(wrenchName), actiForce(actiForce), stopForce(stopForce),
  actiTorque(actiTorque), stopTorque(stopTorque),
  contactId(contactId), X_0_s(X_0_s), maxDist(maxDist), maxRot(maxRot),
  positionTask(positionTask), positionTaskSp(positionTaskSp),
  orientationTask(orientationTask), orientationTaskSp(orientationTaskSp),
  activatedForce(false), activatedTorque(false),
  zVec(X_0_s.rotation().row(2)),
  toRemove(false), targetError(0),
  dof(6, 6)
{
  dof.setZero();
  dof.diagonal().setOnes();
}

bool ActiGripper::update(const mc_rbdyn::Robot & robot,
    tasks::qp::ContactConstr* contactConstr)
{
  const sva::ForceVecd & wrench = robot.forceSensor(wrenchName).wrench();
  double forceNorm = wrench.force().norm();
  //Only consider z-axis torque
  double torqueZ = wrench.couple()(2);
  bool res = updateForce(forceNorm, contactConstr);
  //res = res && updateTorque(torqueZ, contactConstr);
  return res;
}

bool ActiGripper::updateForce(double forceNorm, tasks::qp::ContactConstr* contactConstr)
{
  targetError = positionTask->eval().norm();
  if(!toRemove)
  {
    if(activatedForce)
    {
      if(forceNorm > stopForce && targetError < maxDist*0.1)
      {
        activatedForce = false;
        dof(5, 5) = 1;
        contactConstr->removeDofContact(contactId);
        contactConstr->addDofContact(contactId, dof);
        contactConstr->updateDofContacts();
      }
      else
      {
        double err = std::min(std::max(actiForce - forceNorm, 0.0)/actiForce, 1.0);
        double pos = maxDist*err;
        Eigen::Vector3d target = X_0_s.translation() + zVec*pos;
        positionTask->position(target);
        if(targetError > maxDist*1.5)
        {
          return false;
        }
      }
    }
    else
    {
      if(forceNorm < actiForce)
      {
        // Free translation on z-axis
        dof(5,5) = 0;
        activatedForce = true;
        contactConstr->addDofContact(contactId, dof);
        contactConstr->updateDofContacts();
      }
    }
  }
  else
  {
    Eigen::Vector3d target = X_0_s.translation();
    positionTask->position(target);
  }
  return true;
}

bool ActiGripper::updateTorque(double torqueNorm, tasks::qp::ContactConstr* contactConstr)
{
  targetError = orientationTask->eval().norm();
  if(!toRemove)
  {
    if(activatedTorque)
    {
      if(fabs(torqueNorm) < stopTorque && targetError < maxRot*0.1)
      {
        activatedTorque = false;
        dof(2, 2) = 1;
        contactConstr->removeDofContact(contactId);
        contactConstr->addDofContact(contactId, dof);
        contactConstr->updateDofContacts();
      }
      else
      {
        double err = std::min(std::max(fabs(torqueNorm), 0.0)/(actiTorque), 1.0);
        Eigen::Matrix3d rot = sva::RotZ(copysign(maxRot*err, torqueNorm)); //FIXME: z-axis
        sva::PTransformd move(rot);
        Eigen::Matrix3d target = (move*X_0_s).rotation();
        orientationTask->orientation(target);
        if(targetError > maxRot*1.5)
        {
          return false;
        }
      }
    }
    else
    {
      if(torqueNorm > actiTorque)
      {
        dof(2,2) = 0; //FIXME : z-axis
        activatedTorque = true;
        contactConstr->removeDofContact(contactId);
        contactConstr->addDofContact(contactId, dof);
        contactConstr->updateDofContacts();
      }
    }
  }
  else
  {
    Eigen::Matrix3d target = X_0_s.rotation();
    orientationTask->orientation(target);
  }
  return true;
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
  sch::mc_rbdyn::transform(*(r1hull.get()), X_b1_h1*r1.mbc().bodyPosW[r1BodyIndex]);
  sch::mc_rbdyn::transform(*(r2hull.get()), X_b2_h2*r2.mbc().bodyPosW[r2BodyIndex]);
}

std::vector<mc_rbdyn::Collision> confToColl(const std::vector<mc_rbdyn::StanceConfig::BodiesCollisionConf> & conf)
{
  std::vector<mc_rbdyn::Collision> res;
  for(const auto & bcc : conf)
  {
    res.push_back(mc_rbdyn::Collision(bcc.body1, bcc.body2,
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

MCSeqControllerConfig::MCSeqControllerConfig(const mc_control::Configuration & conf)
{
  if(!conf.has("Seq"))
  {
    LOG_ERROR("No Seq section in configuration file, abort")
    throw("No Seq section in configuration file");
  }
  auto seq = conf("Seq");
  seq("Simulation", is_simulation);
  if(!seq.has("Env"))
  {
    LOG_ERROR("No Env section in configuration for Seq controller, abort")
    throw("No Env section in Seq section");
  }
  auto env = seq("Env");
  if(env.has("Module"))
  {
    env_module = mc_rbdyn::RobotLoader::get_robot_module((std::string)env("Module"));
  }
  else if(env.has("Name"))
  {
    std::string env_path = mc_rtc::MC_ENV_DESCRIPTION_PATH;
    env("Path", env_path);
    std::string env_name = env("Name");
    env_module = mc_rbdyn::RobotLoader::get_robot_module("env", env_path, env_name);
  }
  else
  {
    LOG_ERROR("No Name or Module value for Env in Seq configuration, abort")
    throw("No Name or Module value for Env in Seq configuration");
  }
  if(seq.has("Plan"))
  {
    plan = (std::string)seq("Plan");
    plan = std::string(mc_rtc::DATA_PATH) + "/" + plan;
  }
  else
  {
    LOG_ERROR("No Plan in Seq configuration")
    throw("No Plan in Seq configuration");
  }
  seq("StepByStep", step_by_step);
  seq("UseRealSensors", use_real_sensors);
  seq("StartStance", start_stance);
}

MCSeqController::MCSeqController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt, const MCSeqControllerConfig & config)
: MCController({robot_module, config.env_module}, dt),
  nrIter(0), logger(timeStep),
  publisher(new MCSeqPublisher(robots())),
  step_by_step(config.step_by_step), paused(false), halted(false),
  stanceIndex(config.start_stance), seq_actions(0),
  currentContact(0), targetContact(0), currentGripper(0),
  is_simulation(config.is_simulation),
  use_real_sensors(config.use_real_sensors),
  collsConstraint(robots(), timeStep),
  comIncPlaneConstr(robots(), 0, timeStep),
  max_perc(1.0), nr_points(300),
  samples(0.0, max_perc, nr_points)
{
  logger.logPhase("START", 0);
  /* Load plan */
  loadStances(robots(), config.plan, stances, actions, interpolators);
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
  std::string config_path = config.plan;
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

  constSpeedConstr.reset(new mc_solver::BoundedSpeedConstr(robots(), 0, timeStep));
  qpsolver->addConstraintSet(*constSpeedConstr);

  std::array<double, 3> damper = {0.01, 0.001, 0.};
  dynamicsConstraint = mc_solver::DynamicsConstraint(robots(), 0, timeStep, damper, 0.5);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(collsConstraint);
  qpsolver->addConstraintSet(comIncPlaneConstr);
  qpsolver->setContacts(stances[stanceIndex].geomContacts());

  stabilityTask.reset(new mc_tasks::StabilityTask(robots()));
  solver().addTask(stabilityTask);
  metaTasks.push_back(stabilityTask.get());
  stabilityTask->target(env(), stances[stanceIndex], configs[stanceIndex], configs[stanceIndex].comTask.targetSpeed);

  LOG_SUCCESS("MCSeqController init done")
  LOG_INFO("Setup to play " << seq_actions.size() << " actions")
}

bool MCSeqController::run()
{
  bool ret = true;
  if(!halted && stanceIndex < seq_actions.size())
  {
    ret = MCController::run();
    if(ret && !paused)
    {
      nrIter++;
      unsigned int stanceIndexIn = stanceIndex;
      sensorContacts = contactSensor->update(*this);
      pre_live(); /* pre_live can halt the execution */
      /* Stability polygon interpolation */
      if(startPolygonInterpolator)
      {
        double cur_sample = 0.0; double cur_speed = 0.0;
        samples.next(cur_sample, cur_speed);
        interpol_percent = std::min(cur_sample, max_perc);
        bool done = interpol_percent == max_perc;
        if(!done && stanceIndex < interpolators.size())
        {
          auto clamp = [](const double & v) { return std::min(std::max(v, -0.13), 0.13); };
          auto clamp_pos = [](const double & v) { return std::min(std::max(v, 0.), 0.15); };
          auto poly = interpolators[stanceIndex].fast_interpolate(interpol_percent);
          publisher->publish_poly(poly);
          planes = mc_rbdyn::planes_from_polygon(poly);
          std::vector<Eigen::Vector3d> speeds;
          std::vector<Eigen::Vector3d> nds;
          if(cur_speed != 0)
          {
            auto normal_speeds = interpolators[stanceIndex].normal_derivative(cur_speed/timeStep);
            for(const auto & t : normal_speeds)
            {
              nds.emplace_back(clamp(t[0]), clamp(t[1]), 0);
            }
            auto mid_speeds = interpolators[stanceIndex].midpoint_derivative(cur_speed/timeStep);
            for(const auto & t : mid_speeds)
            {
              speeds.emplace_back(clamp_pos(t[0]), clamp_pos(t[1]), 0.);
            }
          }
          else
          {
            speeds = std::vector<Eigen::Vector3d>(planes.size(), Eigen::Vector3d::Zero());
            nds = speeds;
          }
          if(stanceIndex == 6)
          {
            speeds = std::vector<Eigen::Vector3d>(planes.size(), Eigen::Vector3d::Zero());
            nds = speeds;
          }
          comIncPlaneConstr.set_planes(solver(), planes, speeds, nds);

        }
        else
        {
          std::vector<Eigen::Vector3d> speeds(planes.size(), Eigen::Vector3d::Zero());
          std::vector<Eigen::Vector3d> nds = speeds;
          comIncPlaneConstr.set_planes(solver(), planes, speeds, nds);
          startPolygonInterpolator = false;
        }
      }
      /* Execute the step */
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
            publisher->set_contacts(stances[stanceIndex].geomContacts());
          }
          interpol_percent = 0;
          samples = mc_rbdyn::QuadraticGenerator(0.0, max_perc, nr_points);
          /*FIXME Hackish */
          //if(seq_actions[stanceIndexIn]->type() == mc_control::SeqAction::GripperMove)
          //{
          //  std::vector<std::string> reduce_pdgains_joints = {"LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6"};
          //  for(const auto & jn : reduce_pdgains_joints)
          //  {
          //    double pgain, dgain;
          //    pdgains::getPGain(jn, pgain);
          //    pdgains::getDGain(jn, dgain);
          //    pdgains::setPGain(jn, pgain/2);
          //    pdgains::setDGain(jn, dgain/2);
          //  }
          //}
          paused = step_by_step;
        }
      }
      post_live();
      /* Publish information */
      publisher->publish_com(rbd::computeCoM(robot().mb(), robot().mbc()));
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
  /* Heuristic guess for kinematics vs. dynamics mode */
  if(use_real_sensors)
  {
    robot().mbc().q = reset_data.q;
    robot().mbc().q[0] = stabilityTask->postureTask->posture()[0];
  }
  else
  {
    robot().mbc().q = stabilityTask->postureTask->posture();
  }
  std::cout << "Start at ";
  for(const auto & qi : robot().mbc().q[0])
  {
    std::cout << qi << ", ";
  }
  std::cout << std::endl;
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
  qpsolver->setContacts(stances[stanceIndex].geomContacts());
  publisher->set_contacts(stances[stanceIndex].geomContacts());
  stabilityTask->target(env(), stances[stanceIndex], configs[stanceIndex], configs[stanceIndex].comTask.targetSpeed);
}

std::ostream& MCSeqController::log_header(std::ostream & os)
{
  os << ";stance_index;polygonInterpolatorPercent;filteredForceSensor_fx;filteredForceSensor_fy;filteredForceSensor_fz;filteredForceSensor_cx;filteredForceSensor_cy;filteredForceSensor_cz;comt_x;comt_y;comt_z";
  return os;
}

std::ostream& MCSeqController::log_data(std::ostream & os)
{
  os << ";" << stanceIndex;
  os << ";" << interpol_percent;
  if(complianceTask)
  {
    const auto & w = complianceTask->getFilteredWrench();
    os << ";" << w.force().x()
       << ";" << w.force().y()
       << ";" << w.force().z()
       << ";" << w.couple().x()
       << ";" << w.couple().y()
       << ";" << w.couple().z();
  }
  else
  {
    os << ";0;0;0;0;0;0";
  }
  if(stabilityTask)
  {
    os << ";" << stabilityTask->comObj.x()
       << ";" << stabilityTask->comObj.y()
       << ";" << stabilityTask->comObj.z();
  }
  else
  {
    os << ";0;0;0";
  }
  return os;
}

std::vector<std::string> MCSeqController::supported_robots() const
{
  return {"hrp2_drc"};
}

void MCSeqController::updateRobotEnvCollisions(const std::vector<mc_rbdyn::Contact> & contacts, const mc_rbdyn::StanceConfig & conf)
{
  collsConstraint.setEnvCollisions(solver(), contacts, confToColl(conf.collisions.robotEnv));
}

void MCSeqController::updateSelfCollisions(const std::vector<mc_rbdyn::Contact> & contacts, const mc_rbdyn::StanceConfig & conf)
{
  collsConstraint.setSelfCollisions(solver(), contacts, confToColl(conf.collisions.autoc));
}

void MCSeqController::updateContacts(const std::vector<mc_rbdyn::Contact> & contacts)
{
  qpsolver->setContacts(contacts);

  for(const auto & gTT : gripperTorqueTasks)
  {
    qpsolver->removeTask(gTT.get());
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
      qpsolver->addTask(gTask.get());
    }
  }

  for(const auto & c : contacts)
  {
    mc_rbdyn::GripperSurface * is_gs = dynamic_cast<mc_rbdyn::GripperSurface*>(c.r1Surface().get());
    std::string bodyName = c.r1Surface()->bodyName();
    if(is_gs && actiGrippers.count(bodyName) == 0 && robot().hasForceSensor(bodyName) )
    {
      LOG_INFO("ActiGripper ADD " << bodyName)
      std::string forceSensor = robot().bodyForceSensor(bodyName).name();
      tasks::qp::ContactId contactId = c.contactId(robots());
      sva::PTransformd X_0_s = c.r1Surface()->X_0_s(robot());
      double actiForce = 50; /* FIXME Hard-coded, should at least be an acti gripper const static member */
      double stopForce = 90; /* FIXME ^^ */
      double actiTorque = 2.; /* FIXME ^^ */
      double stopTorque = 1.; /* FIXME ^^ */
      std::shared_ptr<tasks::qp::PositionTask> positionTask(new tasks::qp::PositionTask(robots().mbs(), 0, contactId.r1BodyName, X_0_s.translation(), is_gs->X_b_s().translation()));
      std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp(new tasks::qp::SetPointTask(robots().mbs(), 0, positionTask.get(), 20, 100000.));
      std::shared_ptr<tasks::qp::OrientationTask> orientationTask(new tasks::qp::OrientationTask(robots().mbs(), 0, contactId.r1BodyName, X_0_s.rotation()));
      std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp(new tasks::qp::SetPointTask(robots().mbs(), 0, orientationTask.get(), 20, 100000.));
      qpsolver->addTask(orientationTaskSp.get());
      actiGrippers[bodyName] = ActiGripper(forceSensor, actiForce, stopForce, actiTorque, stopTorque,
          contactId, X_0_s, use_real_sensors ? 0.04:0.01, 0.2, positionTask, positionTaskSp, orientationTask, orientationTaskSp); /*FIXME 0.04 is ActiGripperMaxPull, 5 degrees is ActiGripper::maxRot */
      qpsolver->addTask(positionTaskSp.get());
      qpsolver->addTask(orientationTaskSp.get());
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
      qpsolver->removeTask(ka.second.positionTaskSp.get());
      qpsolver->removeTask(ka.second.orientationTaskSp.get());
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

void MCSeqController::pre_live()
{
  for(auto & ba : actiGrippers)
  {
    /* This cast is guaranted to work */
    tasks::qp::ContactConstr* contactConstr = (dynamic_cast<tasks::qp::ContactConstr*>(contactConstraint.contactConstr.get()));
    if(!ba.second.update(robot(), contactConstr))
    {
      halted = true;
      LOG_ERROR("OOPS TOO MUCH ERROR")
    }
  }
}

void MCSeqController::post_live()
{
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
    ret |= collsConstraint.removeEnvCollisionByBody(solver(), p.first, p.second);
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
  if(paused && stanceIndex < stances.size())
  {
    LOG_INFO("Playing " << actions[stanceIndex]->toStr())
    paused = false;
    return true;
  }
  return false;
}

bool MCSeqController::set_joint_pos(const std::string & jname, const double & pos)
{
  if(robot().hasJoint(jname))
  {
    auto idx = robot().jointIndexByName(jname);
    auto p = stabilityTask->postureTask->posture();
    if(p[idx].size() == 1)
    {
      p[idx][0] = pos;
      stabilityTask->postureTask->posture(p);
      stabilityTask->postureTask->stiffness(2.0);
      return true;
    }
  }
  return false;
}

void MCSeqController::loadStanceConfigs(const std::string & file)
{
  LOG_INFO("Loading stance configs from " << file)
  configs.resize(0);
  mc_rtc::Configuration stanceConfigs(file);
  /*
    The JSON file contains two sections:
    - A General section contains configuration information for the full sequence
    - A StepByStep section contains configuration relative to single steps
  */
  mc_rbdyn::StanceConfig comMoveConfig;
  mc_rbdyn::StanceConfig contactMoveConfig;
  mc_rbdyn::StanceConfig gripperMoveConfig;
  if(stanceConfigs.has("General"))
  {
    auto generalConfig = stanceConfigs("General");
    if(generalConfig.has("CoMMove"))
    {
      mc_rbdyn::StanceConfigFromJSON(comMoveConfig, generalConfig("CoMMove"));
    }
    if(generalConfig.has("ContactMove"))
    {
      mc_rbdyn::StanceConfigFromJSON(contactMoveConfig, generalConfig("ContactMove"));
    }
    if(generalConfig.has("GripperMove"))
    {
      mc_rbdyn::StanceConfigFromJSON(gripperMoveConfig, generalConfig("GripperMove"));
    }
    if(generalConfig.has("Collisions"))
    {
      mc_rbdyn::scCollisionsFromJSON(comMoveConfig.collisions, generalConfig("Collisions"));
      mc_rbdyn::scCollisionsFromJSON(contactMoveConfig.collisions, generalConfig("Collisions"));
      mc_rbdyn::scCollisionsFromJSON(gripperMoveConfig.collisions, generalConfig("Collisions"));
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
        throw("!happenning");
        break;
    }

    /* Look for a matching state in the JSON file */
    if(stanceConfigs.has("StepByStep"))
    {
      auto stepByStep = stanceConfigs("StepByStep");
      std::string type = actions[i]->type();
      std::string r1Surface = "";
      std::string r2Surface = "";
      if(type != "identity")
      {
        r1Surface = actions[i]->contact().r1Surface()->name();
        r2Surface = actions[i]->contact().r2Surface()->name();
      }
      for(size_t i = 0; i <stepByStep.size(); ++i)
      {
        auto scv = stepByStep[i];
        if(scv("type") == type && scv("r1Surface") == r1Surface && scv("r2Surface") == r2Surface)
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
  bool comBranch = (targetIsRemoveContact || targetIsIdentity);
  bool gripperBranch = (curIsRemoveContact && targetIsAddContact && sameSurface && targetIsGripperContact) || (targetIsAddContact && targetIsGripperContact) || (targetIsRemoveContact && targetIsGripperContact && (!targetTargetIsAddContact));

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
                  std::shared_ptr<SeqStep>(new live_CoMOpenGripperT()),
                  std::shared_ptr<SeqStep>(new enter_CoMRemoveGripperT()),
                  std::shared_ptr<SeqStep>(new live_CoMRemoveGripperT()),
                  std::shared_ptr<SeqStep>(new live_moveCoMT()),
                  std::shared_ptr<SeqStep>(new live_CoMCloseGripperT())
    };
    res->_type = SeqAction::CoMMove;
  }
  else
  {
    LOG_ERROR("Could !find a branch for the following action couple:")
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

std::vector<std::string> MC_RTC_CONTROLLER()
{
  return {"Seq"};
}

void destroy(mc_control::MCController * ptr)
{
  delete ptr;
}

mc_control::MCController * create(const std::string &, const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt, const mc_control::Configuration & conf)
{
  return new mc_control::MCSeqController(robot, dt, {conf});
}
