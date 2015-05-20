#include <mc_control/mc_controller.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <fstream>

/* Note all service calls except for controller switches are implemented in mc_drc_controller_services.cpp */

namespace mc_control
{

MCController::MCController(const std::string & env_path, const std::string & env_name)
: MCVirtualController(), robot_module(), env_module(env_path, env_name)
{
  unsigned int hrp2_drc_index = 0;
  {
    /* Entering new scope to prevent access to robots from anywhere but the qpsolver object */
    sva::PTransformd base = sva::PTransformd::Identity();

    mc_rbdyn::Robot hrp2_drc_in;
    mc_rbdyn::Robot env;
    loadRobotAndEnv(robot_module, robot_module.path + "/rsdf/hrp2_drc/",
                  env_module, env_module.path + "/rsdf/" + env_module.name + "/",
                  &base, 0, hrp2_drc_in, env);
    hrp2_drc_in.mbc->gravity = Eigen::Vector3d(0, 0, 9.81);
    mc_rbdyn::Robots robots({hrp2_drc_in, env});



    mc_rbdyn::Robot & hrp2_drc = robots.robot();
    rbd::forwardKinematics(*(hrp2_drc.mb), *(hrp2_drc.mbc));
    rbd::forwardVelocity(*(hrp2_drc.mb), *(hrp2_drc.mbc));


    qpsolver = std::shared_ptr<mc_solver::QPSolver>(new mc_solver::QPSolver(robots, timeStep));
  }
  {
    /* Initiate grippers */
    std::string urdfPath = robot_module.path + "/urdf/hrp2drc.urdf";
    std::ifstream ifs(urdfPath);
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn::Robot urdfRobot = mc_rbdyn::loadRobotFromUrdf(urdf.str());
    lgripper.reset(new Gripper(urdfRobot, "l_gripper", robot(), urdf.str(), 0, timeStep));
    rgripper.reset(new Gripper(urdfRobot, "r_gripper", robot(), urdf.str(), 0, timeStep));
  }

  contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Velocity);

  dynamicsConstraint = mc_solver::DynamicsConstraint(qpsolver->robots, hrp2_drc_index, timeStep,
                                                     false, {0.1, 0.01, 0.5}, 0.5);

  kinematicsConstraint = mc_solver::KinematicsConstraint(qpsolver->robots, hrp2_drc_index, timeStep,
                                                         false, {0.1, 0.01, 0.5}, 0.5);

  selfCollisionConstraint = mc_solver::CollisionsConstraint(qpsolver->robots, hrp2_drc_index, hrp2_drc_index, timeStep);

  /* Give a reasonnable default set of self collisions for the upper body */
  selfCollisionConstraint.addCollisions(qpsolver->robots, {
    mc_solver::Collision("LARM_LINK3", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK4", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK5", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK3", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK4", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK5", "BODY", 0.05, 0.01, 0.),
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

void MCController::reset(const ControllerResetData & reset_data)
{
  robot().mbc->zero(*(robot().mb));
  robot().mbc->q = reset_data.q;
  postureTask->posture(reset_data.q);
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));
  qpsolver->setContacts({
  });
}

void MCController::setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches)
{
  this->wrenches = wrenches;
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

const mc_rbdyn::Robots & MCController::robots() const
{
  return qpsolver->robots;
}

mc_rbdyn::Robots & MCController::robots()
{
  return qpsolver->robots;
}

}
