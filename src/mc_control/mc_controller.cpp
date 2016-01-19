#include <mc_control/mc_controller.h>

#include <mc_rbdyn/RobotLoader.h>

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <fstream>

namespace mc_control
{

MCController::MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt)
: MCController({robot, mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"))}, dt)
{
}

MCController::MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots_modules, double dt)
: timeStep(dt)
{
  /* Load the bots and initialize QP solver instance */
  {
  std::vector<std::string> surfaceDirs;
  for(const auto & m : robots_modules)
  {
    surfaceDirs.push_back(m->rsdf_dir);
  }
  auto robots = mc_rbdyn::loadRobots(robots_modules, surfaceDirs);
  for(auto & robot: robots->robots())
  {
    robot.mbc().gravity = Eigen::Vector3d(0, 0, 9.81);
    rbd::forwardKinematics(robot.mb(), robot.mbc());
    rbd::forwardVelocity(robot.mb(), robot.mbc());
  }
  qpsolver.reset(new mc_solver::QPSolver(robots, timeStep));
  }

  /* Initialize grippers */
  {
  std::string urdfPath = robots_modules[0]->urdf_path;
  std::ifstream ifs(urdfPath);
  if(ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    auto urdfRobot = mc_rbdyn::loadRobotFromUrdf("temp_robot", urdf.str());
    lgripper.reset(new Gripper(urdfRobot->robot(), "l_gripper", robot(), urdf.str(), 0, timeStep));
    rgripper.reset(new Gripper(urdfRobot->robot(), "r_gripper", robot(), urdf.str(), 0, timeStep));
  }
  else
  {
    LOG_ERROR("Could not open urdf file " << urdfPath << " for robot " << robots_modules[0]->name << ", cannot initialize grippers")
    throw("Failed to initialize grippers");
  }
  }

  /* Initialize constraints and tasks */
  contactConstraint = mc_solver::ContactConstraint(timeStep, mc_solver::ContactConstraint::Velocity);
  dynamicsConstraint = mc_solver::DynamicsConstraint(robots(), 0, timeStep,
                                                  false, {0.1, 0.01, 0.5}, 0.5);
  kinematicsConstraint = mc_solver::KinematicsConstraint(robots(), 0, timeStep,
                                                  false, {0.1, 0.01, 0.5}, 0.5);
  selfCollisionConstraint = mc_solver::CollisionsConstraint(robots(), 0, 0, timeStep);
  selfCollisionConstraint.addCollisions(robots(), robots_modules[0]->defaultSelfCollisions());
  postureTask.reset(new tasks::qp::PostureTask(robots().mbs(), 0, robot().mbc().q, 10, 5));
  LOG_INFO("MCController(base) ready")
}

MCController::~MCController()
{
}

bool MCController::run()
{
  if(!qpsolver->run())
  {
    LOG_ERROR("QP failed to run()")
    return false;
  }
  return true;
}

const QPResultMsg & MCController::send(const double & t)
{
  return qpsolver->send(t);
}

void MCController::reset(const ControllerResetData & reset_data)
{
  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  postureTask->posture(reset_data.q);
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
}

void MCController::setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches)
{
  this->wrenches = wrenches;
}

const mc_rbdyn::Robot & MCController::robot() const
{
  return qpsolver->robots->robot();
}

const mc_rbdyn::Robot & MCController::env() const
{
  return qpsolver->robots->env();
}

mc_rbdyn::Robot & MCController::robot()
{
  return qpsolver->robots->robot();
}

mc_rbdyn::Robot & MCController::env()
{
  return qpsolver->robots->env();
}

const mc_rbdyn::Robots & MCController::robots() const
{
  return *(qpsolver->robots);
}

mc_rbdyn::Robots & MCController::robots()
{
  return *(qpsolver->robots);
}

bool MCController::set_joint_pos(const std::string & jname, const double & pos)
{
  if(robot().hasJoint(jname))
  {
    auto idx = robot().jointIndexByName(jname);
    auto p = postureTask->posture();
    if(p[idx].size() == 1)
    {
      p[idx][0] = pos;
      postureTask->posture(p);
      return true;
    }
  }
  return false;
}

bool MCController::change_ef(const std::string &)
{
  return false;
}

bool MCController::move_ef(const Eigen::Vector3d &, const Eigen::Matrix3d &)
{
  return false;
}

bool MCController::move_com(const Eigen::Vector3d &)
{
  return false;
}

bool MCController::play_next_stance()
{
  return false;
}

bool MCController::driving_service(double, double, double, double)
{
  return false;
}

bool MCController::read_msg(std::string &)
{
  return false;
}

bool MCController::read_write_msg(std::string &, std::string &)
{
  return true;
}

std::ostream& MCController::log_header(std::ostream & os)
{
  return os;
}

std::ostream& MCController::log_data(std::ostream & os)
{
  return os;
}

std::vector<std::string> MCController::supported_robots() const
{
  return {};
}

}
