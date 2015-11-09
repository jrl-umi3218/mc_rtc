#include <mc_control/mc_mrqp_controller.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <fstream>

namespace mc_control
{

MCMRQPController::MCMRQPController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & env_modules)
: MCVirtualController(), robot_modules(env_modules)
{
  robot_modules.insert(robot_modules.begin(), std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::HRP2DRCGripperRobotModule()));
  unsigned int hrp2_drc_index = 0;
  {
    /* Entering new scope to prevent access to robots from anywhere but the qpsolver object */
    std::vector<std::string> surfaceDirs;
    for(const auto & m : robot_modules)
    {
      surfaceDirs.push_back(m->path + "/rsdf/" + m->name + "/");
    }

    mc_rbdyn::Robots robots = mc_rbdyn::loadRobots(robot_modules, surfaceDirs);

    for(auto & robot: robots.robots())
    {
      robot.mbc().gravity = Eigen::Vector3d(0, 0, 9.81);
      rbd::forwardKinematics(robot.mb(), robot.mbc());
      rbd::forwardVelocity(robot.mb(), robot.mbc());
    }

    mrqpsolver.reset(new mc_solver::QPSolver(robots, timeStep));
  }
  {
    /* Initiate grippers */
    std::string urdfPath = robot_module.path + "/urdf/hrp2drc.urdf";
    std::ifstream ifs(urdfPath);
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn::Robots urdfRobot = mc_rbdyn::loadRobotFromUrdf("temp_hrp2_drc", urdf.str());
    lgripper.reset(new Gripper(urdfRobot.robot(), "l_gripper", robot(), urdf.str(), 0, timeStep));
    rgripper.reset(new Gripper(urdfRobot.robot(), "r_gripper", robot(), urdf.str(), 0, timeStep));
  }
  hrp2contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Velocity);
  hrp2dynamicsConstraint = mc_solver::DynamicsConstraint(mrqpsolver->robots, hrp2_drc_index, timeStep,
                                                     false, {0.1, 0.01, 0.1}, 0.5);
  hrp2kinematicsConstraint = mc_solver::KinematicsConstraint(mrqpsolver->robots, hrp2_drc_index, timeStep,
                                                         false, {0.1, 0.01, 1e-2}, 0.5);
  hrp2selfCollisionConstraint = mc_solver::CollisionsConstraint(mrqpsolver->robots, hrp2_drc_index, hrp2_drc_index, timeStep);

  /* Give a reasonnable default set of self collisions for the upper body */
  hrp2selfCollisionConstraint.addCollisions(mrqpsolver->robots, {
    mc_solver::Collision("LARM_LINK2", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK3", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK4", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK5", "BODY", 0.05, 0.01, 0.),

    mc_solver::Collision("RARM_LINK2", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK3", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK4", "BODY", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK5", "BODY", 0.05, 0.01, 0.),

    mc_solver::Collision("RARM_LINK2", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK3", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("RARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.),

    mc_solver::Collision("LARM_LINK2", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK3", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_solver::Collision("LARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.)
  });

  hrp2postureTask = std::shared_ptr<tasks::qp::PostureTask>(new tasks::qp::PostureTask(mrqpsolver->robots.mbs(), static_cast<int>(hrp2_drc_index), mrqpsolver->robots.robot().mbc().q, 1, 5));
  std::cout << "MCController(base) ready" << std::endl;
}

bool MCMRQPController::run()
{
  if(!mrqpsolver->run())
  {
    std::cerr << "MRQP failed to run()" << std::endl;
    return false;
  }
  return true;
}

const QPResultMsg & MCMRQPController::send(const double & t)
{
  return mrqpsolver->send(t);
}

void MCMRQPController::reset(const ControllerResetData & reset_data)
{
  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  hrp2postureTask->posture(reset_data.q);
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
  mrqpsolver->setContacts({
  });
}

void MCMRQPController::setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches)
{
  this->wrenches = wrenches;
}

const mc_rbdyn::Robot & MCMRQPController::robot() const
{
  return mrqpsolver->robots.robot();
}

const mc_rbdyn::Robot & MCMRQPController::env() const
{
  return mrqpsolver->robots.env();
}

const mc_rbdyn::Robots & MCMRQPController::robots() const
{
  return mrqpsolver->robots;
}

mc_rbdyn::Robots & MCMRQPController::robots()
{
  return mrqpsolver->robots;
}

mc_rbdyn::Robot & MCMRQPController::robot()
{
  return mrqpsolver->robots.robot();
}

mc_rbdyn::Robot & MCMRQPController::env()
{
  return mrqpsolver->robots.env();
}

bool MCMRQPController::joint_up(const std::string & jname)
{
  if(robot().hasJoint(jname))
  {
    auto idx = robot().jointIndexByName(jname);
    auto p = hrp2postureTask->posture();
    if(p[idx].size() == 1)
    {
      p[idx][0] += 0.01;
      hrp2postureTask->posture(p);
      return true;
    }
  }
  return false;
}

bool MCMRQPController::joint_down(const std::string & jname)
{
  if(robot().hasJoint(jname))
  {
    auto idx = robot().jointIndexByName(jname);
    auto p = hrp2postureTask->posture();
    if(p[idx].size() == 1)
    {
      p[idx][0] -= 0.01;
      hrp2postureTask->posture(p);
      return true;
    }
  }
  return false;
}

bool MCMRQPController::set_joint_pos(const std::string & jname, const double & pos)
{
  if(robot().hasJoint(jname))
  {
    auto idx = robot().jointIndexByName(jname);
    auto p = hrp2postureTask->posture();
    if(p[idx].size() == 1)
    {
      p[idx][0] = pos;
      hrp2postureTask->posture(p);
      return true;
    }
  }
  return false;
}

}
