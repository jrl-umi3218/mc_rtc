#include <mc_control/mc_drc_controller.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_drc_controller_services.cpp */

namespace mc_control
{

EndEffectorTask::EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex)
: bodyName(bodyName)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];
  unsigned int bodyId = robot.bodyIdByName(bodyName);
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd bpw = robot.mbc->bodyPosW[bodyIndex];

  curTransform = bpw;

  positionTask.reset(new tasks::qp::PositionTask(robots.mbs, robotIndex, bodyId, bpw.translation(), Eigen::Vector3d(0,0,0)));
  positionTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs, robotIndex, positionTask.get(), 2, 1000));

  orientationTask.reset(new tasks::qp::OrientationTask(robots.mbs, robotIndex, bodyId, bpw.rotation()));
  orientationTaskSp.reset(new tasks::qp::SetPointTask(robots.mbs, robotIndex, orientationTask.get(), 2, 1000));
}

void EndEffectorTask::resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);

  curTransform = robot.mbc->bodyPosW[bodyIndex];
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

void EndEffectorTask::removeFromSolver(mc_solver::QPSolver & qpsolver)
{
  qpsolver.solver.removeTask(positionTaskSp.get());
  qpsolver.solver.removeTask(orientationTaskSp.get());
  qpsolver.update();
}

void EndEffectorTask::addToSolver(mc_solver::QPSolver & qpsolver)
{
  qpsolver.solver.addTask(positionTaskSp.get());
  qpsolver.solver.addTask(orientationTaskSp.get());
  qpsolver.update();
}

void EndEffectorTask::add_ef_pose(const sva::PTransformd & dtr)
{
  Eigen::Matrix3d new_rot = curTransform.rotation()*dtr.rotation();
  Eigen::Vector3d new_t = curTransform.translation() + dtr.translation();
  curTransform = sva::PTransformd(new_rot, new_t);
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

MCController::MCController()
: timeStep(0.005), robot_module(), ground_module()
{
  unsigned int hrp2_drc_index = 0;
  {
    /* Entering new scope to prevent access to robots from anywhere but the qpsolver object */
    sva::PTransformd base = sva::PTransformd::Identity();

    mc_rbdyn::Robot hrp2_drc_in;
    mc_rbdyn::Robot ground;
    loadRobotAndEnv(robot_module, robot_module.path + "/rsdf/hrp2_drc/",
                  ground_module, ground_module.path + "/rsdf/ground/",
                  &base, 0, hrp2_drc_in, ground);
    hrp2_drc_in.mbc->gravity = Eigen::Vector3d(0, 0, 9.81);
    mc_rbdyn::Robots robots({hrp2_drc_in, ground});



    mc_rbdyn::Robot & hrp2_drc = robots.robot();
    rbd::forwardKinematics(*(hrp2_drc.mb), *(hrp2_drc.mbc));
    rbd::forwardVelocity(*(hrp2_drc.mb), *(hrp2_drc.mbc));


    qpsolver = std::shared_ptr<mc_solver::QPSolver>(new mc_solver::QPSolver(robots, timeStep));
  }

  contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Position);

  kinematicsConstraint = mc_solver::KinematicsConstraint(qpsolver->robots, hrp2_drc_index, timeStep,
                                                         false, {0.1, 0.01, 0.5}, 0.5);

  selfCollisionConstraint = mc_solver::CollisionsConstraint(qpsolver->robots, hrp2_drc_index, hrp2_drc_index, timeStep);

  /* Give a reasonnable default set of self collisions for the upper body */
  selfCollisionConstraint.addCollisions(qpsolver->robots, {
    //FIXME Collision with BODY seems bugged right now
    //mc_solver::Collision("LARM_LINK3", "BODY", 0.05, 0.01, 0.),
    //mc_solver::Collision("LARM_LINK4", "BODY", 0.05, 0.01, 0.),
    //mc_solver::Collision("LARM_LINK5", "BODY", 0.05, 0.01, 0.),
    //mc_solver::Collision("RARM_LINK3", "BODY", 0.05, 0.01, 0.),
    //mc_solver::Collision("RARM_LINK4", "BODY", 0.05, 0.01, 0.),
    //mc_solver::Collision("RARM_LINK5", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK3", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK4", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK5", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK3", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK4", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK5", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.)
  });

  postureTask = std::shared_ptr<tasks::qp::PostureTask>(new tasks::qp::PostureTask(qpsolver->robots.mbs, hrp2_drc_index, qpsolver->robots.robot().mbc->q, 1, 5));
  std::cout << "MCController(base) ready" << std::endl;

}

bool MCController::run()
{
  if(!qpsolver->run())
  {
    std::cerr << "QP failed to run()" << std::endl;
    return false;
  }
  return true;
}

const mc_control::QPResultMsg & MCController::send(const double & t)
{
  return qpsolver->send(t);
}

void MCController::reset(const std::vector< std::vector<double> > & q)
{
  robot().mbc->zero(*(robot().mb));
  robot().mbc->q = q;
  postureTask->posture(q);
  rbd::eulerIntegration(*(robot().mb), *(robot().mbc), qpsolver->timeStep);
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));
}

const mc_rbdyn::Robot & MCController::robot() const
{
  return qpsolver->robots.robot();
}

const mc_rbdyn::Robot & MCController::env() const
{
  return qpsolver->robots.env();
}

mc_rbdyn::Robot & MCController::robot()
{
  return qpsolver->robots.robot();
}

mc_rbdyn::Robot & MCController::env()
{
  return qpsolver->robots.env();
}

/* Common stuff */
MCDRCPostureController::MCDRCPostureController()
: MCController(), current_joint(1)
{
  qpsolver->setContacts({});

  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->solver.addTask(postureTask.get());

  std::cout << "MCDRCPostureController init done" << std::endl;
}

MCDRCBody6dController::MCDRCBody6dController()
: MCController()
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->solver.addTask(postureTask.get());
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("Butthock"), env().surfaces.at("AllGround"))
  });

  std::cout << "MCDRCBody6dController init done" << std::endl;
  efTask.reset(new EndEffectorTask("RARM_LINK6", qpsolver->robots, qpsolver->robots.robotIndex));
  efTask->addToSolver(*qpsolver);
}

void MCDRCBody6dController::reset(const std::vector< std::vector<double> > & q)
{
  MCController::reset(q);
  efTask->resetTask(qpsolver->robots, qpsolver->robots.robotIndex);
}

MCDRCGlobalController::MCDRCGlobalController()
: posture_controller(), body6d_controller(),
  current_ctrl(POSTURE), next_ctrl(POSTURE),
  controller(&posture_controller),
  //current_ctrl(BODY6D), next_ctrl(BODY6D),
  //controller(&body6d_controller),
  next_controller(0)
{
}

void MCDRCGlobalController::init(const std::vector<double> & initq)
{
  std::vector<std::vector<double>> q;
  /*FIXME Get the position/attitude of the robot? */
  q.push_back({1,0,0,0,0,0,0});

  /* The OpenRTM components don't give q in the same order as the QP */
  for(size_t i = 0; i < 24; ++i) // until RARM_LINK7
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 32; i < 37; ++i) // RHAND
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 24; i < 32; ++i) // LARM_LINK*
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 37; i < 42; ++i) // LHAND
  {
    q.push_back({initq[i]});
  }
  controller->reset(q);
}

bool MCDRCGlobalController::run()
{
  /* Check if we need to change the controller this time */
  if(next_controller)
  {
    std::cout << "Switching controllers" << std::endl;
    if(!running)
    {
      controller = next_controller;
    }
    else
    {
      /*XXX Need to be careful here */
      next_controller->reset(controller->robot().mbc->q);
      controller = next_controller;
    }
    next_controller = 0;
    current_ctrl = next_ctrl;
  }
  if(running)
  {
    bool r = controller->run();
    return r;
  }
  else
  {
    return false;
  }
}

const mc_control::QPResultMsg & MCDRCGlobalController::send(const double & t)
{
  return controller->send(t);
}

const mc_solver::QPSolver & MCDRCGlobalController::qpsolver() const
{
  return *(controller->qpsolver);
}

bool MCDRCGlobalController::EnablePostureController()
{
  next_ctrl = POSTURE;
  if(current_ctrl != POSTURE)
  {
    next_controller = &posture_controller;
  }
  //while(next_controller != 0);
  return true;
}

bool MCDRCGlobalController::EnableBody6dController()
{
  next_ctrl = BODY6D;
  if(current_ctrl != BODY6D)
  {
    next_controller = &body6d_controller;
  }
  //while(next_controller != 0);
  return true;
}

}
