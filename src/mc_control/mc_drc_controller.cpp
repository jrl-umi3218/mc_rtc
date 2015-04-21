#include <mc_control/mc_drc_controller.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

EndEffectorTask::EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots & robots, unsigned int robotIndex)
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
: timeStep(0.005), running(false), robot_module(), ground_module()
{
  std::cout << "Creating an MCController" << std::endl;
  sva::PTransformd base = sva::PTransformd::Identity();

  mc_rbdyn::Robot hrp2_drc_in;
  mc_rbdyn::Robot ground;
  loadRobotAndEnv(robot_module, robot_module.path + "/rsdf/hrp2_drc/",
                ground_module, ground_module.path + "/rsdf/ground/",
                &base, 0, hrp2_drc_in, ground);
  std::cout << "Loaded the robot" << std::endl;
  hrp2_drc_in.mbc->gravity = Eigen::Vector3d(0, 0, 9.81);
  mc_rbdyn::Robots robots({hrp2_drc_in, ground});


  unsigned int hrp2_drc_index = 0;

  mc_rbdyn::Robot & hrp2_drc = robots.robot();
  rbd::forwardKinematics(*(hrp2_drc.mb), *(hrp2_drc.mbc));
  rbd::forwardVelocity(*(hrp2_drc.mb), *(hrp2_drc.mbc));


  std::cout << "Create a QPSolver" << std::endl;
  qpsolver = std::shared_ptr<mc_solver::QPSolver>(new mc_solver::QPSolver(robots, timeStep));

  std::cout << "Create a ContactConstraint" << std::endl;
  contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Position);

  std::cout << "Create a KinematicsConstraint" << std::endl;
  kinematicsConstraint = mc_solver::KinematicsConstraint(robots, hrp2_drc_index, timeStep,
                                                         false, {0.1, 0.01, 0.5}, 0.5);

  std::cout << "Create a CollisionsConstraint" << std::endl;
  selfCollisionConstraint = mc_solver::CollisionsConstraint(robots, hrp2_drc_index, hrp2_drc_index, timeStep);

  std::cout << "Add collisions to CollisionsConstraint" << std::endl;
  /* Give a reasonnable default set of self collisions for the upper body */
  selfCollisionConstraint.addCollisions(robots, {
    mc_solver::Collision("RARM_LINK3", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK4", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK5", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK3", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK4", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK5", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK3", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK4", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK5", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK3", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK4", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK5", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.)
  });

  std::cout << "Create a PostureTask" << std::endl;
  postureTask = std::shared_ptr<tasks::qp::PostureTask>(new tasks::qp::PostureTask(qpsolver->robots.mbs, hrp2_drc_index, qpsolver->robots.robot().mbc->q, 1, 5));
  std::cout << "MCController(base) ready" << std::endl;

}

bool MCController::run()
{
  if(running)
  {
    if(!qpsolver->run())
    {
      std::cerr << "QP failed to run()" << std::endl;
      return false;
    }
    return true;
  }
  return false;
}

const mc_control::QPResultMsg & MCController::send(const double & t)
{
  return qpsolver->send(t);
}

const mc_rbdyn::Robot & MCController::robot() const
{
  return qpsolver->robots.robot();
}

const mc_rbdyn::Robot & MCController::env() const
{
  return qpsolver->robots.env();
}

/* Common stuff */
MCDRCPostureController::MCDRCPostureController()
: current_joint(1)
{
  std::cout << "Setup a MCDRCPostureController" << std::endl;
  std::cout << "Set contacts on QPSolver" << std::endl;
  qpsolver->setContacts({});

  //qpsolver->addConstraintSet(contactConstraint);
  //qpsolver->addConstraintSet(kinematicsConstraint);
  //qpsolver->addConstraintSet(selfCollisionConstraint);
  std::cout << "Add task to QPSolver" << std::endl;
  qpsolver->solver.addTask(postureTask.get());

  std::cout << "MCDRCController init done" << std::endl;
}

/* Specific to posture controller */
bool MCDRCPostureController::change_joint(int jid)
{
  if(jid < qpsolver->robots.robot().mb->nrJoints())
  {
    current_joint = jid + 1;
    return true;
  }
  else
  {
    std::cerr << "Invalid joint id given, control unchanged" << std::endl;
    return false;
  }
}
bool MCDRCPostureController::change_joint(const std::string & jname)
{
  if(qpsolver->robots.robot().hasJoint(jname))
  {
    current_joint = qpsolver->robots.robot().jointIndexByName(jname);
    return true;
  }
  else
  {
    std::cerr << "Invalid joint name: " << jname << ", control unchanged" << std::endl;
    return false;
  }
}

bool MCDRCPostureController::joint_up()
{
  add_joint_pos(0.01);
  return true;
}

bool MCDRCPostureController::joint_down()
{
  add_joint_pos(-0.01);
  return true;
}

void MCDRCPostureController::add_joint_pos(const double & v)
{
  auto p = postureTask->posture();
  p[current_joint][0] += v;
  postureTask->posture(p);
}

bool MCDRCPostureController::set_joint_pos(const std::string & jname, const double & v)
{
  if(qpsolver->robots.robot().hasJoint(jname))
  {
    unsigned int jid = qpsolver->robots.robot().jointIndexByName(jname);
    auto p = postureTask->posture();
    p[jid][0] = v;
    postureTask->posture(p);
    return true;
  }
  else
  {
    std::cerr << "Invalid joint name " << jname << " provided" << std::endl;
    return false;
  }
}


MCDRCBody6dController::MCDRCBody6dController()
{
  std::cout << "Setup a MCDRCBody6dController" << std::endl;
  std::cout << "Add contact constraint" << std::endl;
  qpsolver->addConstraintSet(contactConstraint);
  std::cout << "Add kinematics constraints" << std::endl;
  qpsolver->addConstraintSet(kinematicsConstraint);
  std::cout << "Add self collision constraints" << std::endl;
  qpsolver->addConstraintSet(selfCollisionConstraint);
  std::cout << "Add posture task" << std::endl;
  qpsolver->solver.addTask(postureTask.get());
  std::cout << "Set contacts" << std::endl;
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("Butthock"), env().surfaces.at("AllGround"))
  });

  std::cout << "Initiate EF task" << std::endl;
  efTask.reset(new EndEffectorTask("RARM_LINK6", qpsolver->robots, qpsolver->robots.robotIndex));
  std::cout << "Add EF task to solver" << std::endl;
  efTask->addToSolver(*qpsolver);
  std::cout << "Setup done" << std::endl;
}

bool MCDRCBody6dController::change_ef(const std::string & ef_name)
{
  if(robot().hasBody(ef_name))
  {
    efTask->removeFromSolver(*qpsolver);
    postureTask->posture(robot().mbc->q);
    efTask.reset(new EndEffectorTask(ef_name, qpsolver->robots, qpsolver->robots.robotIndex));
    efTask->addToSolver(*qpsolver);
    return true;
  }
  else
  {
    std::cerr << "Invalid link name: " << ef_name << ", control unchanged" << std::endl;
    return false;
  }
}

bool MCDRCBody6dController::translate_ef(const Eigen::Vector3d & t)
{
  sva::PTransformd dtr(Eigen::Matrix3d::Identity(), t);
  efTask->add_ef_pose(dtr);
  return true;
}

bool MCDRCBody6dController::rotate_ef(const Eigen::Matrix3d & m)
{
  sva::PTransformd dtr(m, Eigen::Vector3d(0,0,0));
  efTask->add_ef_pose(dtr);
  return true;
}

}
