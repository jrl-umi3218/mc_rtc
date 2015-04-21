#include <mc_control/mc_drc_controller.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

/* Common stuff */
MCDRCPostureController::MCDRCPostureController()
: running(false), timeStep(0.005), robot_module(), ground_module(), current_joint(1)
{
  std::cout << "Creating MCDRCPostureController" << std::endl;
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

  //std::cout << "Create a ContactConstraint" << std::endl;
  //contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Position);

  //std::cout << "Create a KinematicsConstraint" << std::endl;
  //kinematicsConstraint = mc_solver::KinematicsConstraint(robots, hrp2_drc_index, timeStep,
  //                                                       false, {0.1, 0.01, 0.5}, 0.5);

  //std::cout << "Create a CollisionsConstraint" << std::endl;
  //selfCollisionConstraint = mc_solver::CollisionsConstraint(robots, hrp2_drc_index, hrp2_drc_index, timeStep);

  //std::cout << "Add collisions to CollisionsConstraint" << std::endl;
  //selfCollisionConstraint.addCollisions(robots, {
  //  mc_solver::Collision("RARM_LINK3", "BODY", 0.05, 0.01, 0.),
  //  mc_solver::Collision("RARM_LINK4", "BODY", 0.05, 0.01, 0.),
  //  mc_solver::Collision("RARM_LINK5", "BODY", 0.05, 0.01, 0.),
  //  mc_solver::Collision("RARM_LINK3", "CHEST_LINK0", 0.05, 0.01, 0.),
  //  mc_solver::Collision("RARM_LINK4", "CHEST_LINK0", 0.05, 0.01, 0.),
  //  mc_solver::Collision("RARM_LINK5", "CHEST_LINK0", 0.05, 0.01, 0.),
  //  mc_solver::Collision("RARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
  //  mc_solver::Collision("RARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.),
  //  mc_solver::Collision("LARM_LINK3", "BODY", 0.05, 0.01, 0.),
  //  mc_solver::Collision("LARM_LINK4", "BODY", 0.05, 0.01, 0.),
  //  mc_solver::Collision("LARM_LINK5", "BODY", 0.05, 0.01, 0.),
  //  mc_solver::Collision("LARM_LINK3", "CHEST_LINK0", 0.05, 0.01, 0.),
  //  mc_solver::Collision("LARM_LINK4", "CHEST_LINK0", 0.05, 0.01, 0.),
  //  mc_solver::Collision("LARM_LINK5", "CHEST_LINK0", 0.05, 0.01, 0.),
  //  mc_solver::Collision("LARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
  //  mc_solver::Collision("LARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.)
  //});

  std::cout << "Create a PostureTask" << std::endl;
  postureTask = std::shared_ptr<tasks::qp::PostureTask>(new tasks::qp::PostureTask(qpsolver->robots.mbs, hrp2_drc_index, qpsolver->robots.robot().mbc->q, 1, 5));

  std::cout << "Set contacts on QPSolver" << std::endl;
  qpsolver->setContacts({});

  //qpsolver->addConstraintSet(contactConstraint);
  //qpsolver->addConstraintSet(kinematicsConstraint);
  //qpsolver->addConstraintSet(selfCollisionConstraint);
  std::cout << "Add task to QPSolver" << std::endl;
  qpsolver->solver.addTask(postureTask.get());

  std::cout << "MCDRCController init done" << std::endl;
}

bool MCDRCPostureController::run()
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

const mc_control::QPResultMsg & MCDRCPostureController::send(const double & t)
{
  return qpsolver->send(t);
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

}
