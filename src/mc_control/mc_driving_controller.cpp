#include "mc_driving_controller.h"

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Surface.h>

#include <mc_rtc/logging.h>

#include <Tasks/QPContactConstr.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#ifndef WIN32
#include <sys/time.h>
#else
#include <stdint.h>
#include <Windows.h>
# if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#  define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
# else
#  define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
# endif
inline void gettimeofday(struct timeval *tv, void *)
{
  FILETIME ft;
  unsigned __int64 tmpres = 0;
  static int tzflag;

  if (NULL != tv)
  {
    GetSystemTimeAsFileTime(&ft);

    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;

    /*converting file time to unix epoch*/
    tmpres /= 10;  /*convert into microseconds*/
    tmpres -= DELTA_EPOCH_IN_MICROSECS;
    tv->tv_sec = (long)(tmpres / 1000000UL);
    tv->tv_usec = (long)(tmpres % 1000000UL);
  }
}

#endif

namespace mc_control
{

MCDrivingController::MCDrivingController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
  : MCController({robot_module, mc_rbdyn::RobotLoader::get_robot_module("PolarisRanger", true)}, dt),
    graspOffset(sva::RotX(-M_PI/2), Eigen::Vector3d(0., 0., 0.)),
    ef_task("RARM_LINK6", robots(), 0),
    drivingContacts(),
    collsConstraint(robots(), 0, 1, timeStep),
    iter_(0), theta_(0),
    logging_(false), log_ankle_(), log_wheel_(),
    log_ef_(), log_acc_(), log_rpy_(),
    tMax_(0.5), tMin_(-0.5),
    head_task("HEAD_LINK1", robots(), 0),
    lhand_task("LARM_LINK6", robots(), 0)
{
  std::array<double, 3> damper = {0.1, 0.01, 0.01};
  polarisKinematicsConstraint = mc_solver::KinematicsConstraint(robots(), 1, timeStep, damper, 0.5),
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(kinematicsConstraint);
  qpsolver->addConstraintSet(polarisKinematicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addConstraintSet(collsConstraint);

  std::vector<tasks::qp::JointStiffness> jsv;
  jsv.push_back({"RLEG_JOINT4", 100.});
  postureTask->jointsStiffness(robots().mbs(), jsv);
  qpsolver->addTask(postureTask.get());

  //robot().mbc().q[0] = {0.8018680589369662, 0.09936561148509283, -0.06541812773434774, 0.5855378381237102, -0.3421374123035909, -0.0002850914593993392, 0.8847053544605464};
  robot().mbc().q[0] = {1, 0, 0, 0, 0, 0, 0.76};

  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  drivingContacts.emplace_back(robots(), 0, 1,
                           "Butthock", "left_seat");
  drivingContacts.emplace_back(robots(), 0, 1,
                           "LFullSole", "exit_platform");
  drivingContacts.emplace_back(robots(), 0, 1,
                           "LeftThight", "left_seat");
  drivingContacts.emplace_back(robots(), 0, 1,
                           "RightThight", "left_seat");
  drivingContacts.emplace_back(robots(), 0, 1,
                           "RightGripper", "bar_wheel");
  //drivingContacts.emplace_back(robots().robotIndex, 1,
  //                         robot().surfaces.at("LowerBack"),
  //                         polaris.surfaces.at("left_back"));
  qpsolver->setContacts(drivingContacts);
  //collsConstraint.addCollision(robots(),
  //  mc_rbdyn::Collision("CHEST_LINK1", "seat_back", 0.4, 0.25, 0.0)
  //);

  solver().addTask(&ef_task);
  solver().removeTask(&ef_task);

  polarisPostureTask = std::shared_ptr<tasks::qp::PostureTask>(new tasks::qp::PostureTask(robots().mbs(), 1, robots().robot(1).mbc().q, 5, 100));

  LOG_SUCCESS("MCDrivingController init done")
}

bool MCDrivingController::run()
{
  bool success = MCController::run();
  if(logging_)
  {
    mc_rbdyn::Robot & polaris = robots().robot(1);
    unsigned int wheel_i = robots().robot(1).jointIndexByName("steering_joint");
    struct timeval tv;
    gettimeofday(&tv, 0);
    uint64_t t = static_cast<uint64_t>(tv.tv_sec)*1000000 + static_cast<uint64_t>(tv.tv_usec);
    log_ankle_ << t << " " << robot().mbc().q[robot().jointIndexByName("RLEG_JOINT4")][0] << " " << theta_ << std::endl;
    log_wheel_ << t << " " << polaris.mbc().q[wheel_i][0] << " " << polarisPostureTask->posture()[wheel_i][0] << std::endl;
    const auto & X_0_hand = robot().mbc().bodyPosW[robot().bodyIndexByName("RARM_LINK6")];
    Eigen::Quaterniond ori(X_0_hand.rotation());
    ori = ori.inverse();
    const auto & v = X_0_hand.translation();
    log_ef_ << t << " " << ori.w() << " " << ori.x() << " " << ori.y() << ori.z() << " " << v.transpose() << std::endl;
    log_acc_ << t << " " << robot().bodySensor().acceleration().transpose() << std::endl;
    log_rpy_ << t << " " << robot().bodySensor().orientation().vec().transpose() << std::endl;
  }
  iter_++;
  return success;
}

void MCDrivingController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  LOG_INFO("Enter reset")
  robot().mbc().zero(robot().mb());
  robot().mbc().q = reset_data.q;
  //robot().mbc().q[0] = {0.8018680589369662, 0.09936561148509283, -0.06541812773434774, 0.5855378381237102, -0.3421374123035909, -0.0002850914593993392, 0.8847053544605464};
  robot().mbc().q[0] = {1, 0, 0, 0, 0, 0, 0.76};
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
  postureTask->posture(robot().mbc().q);

  resetWheelTransform();
  //resetBasePose();

  /*std::cout << sva::transformError(gripperSurface->X_0_s(robot(), robot().mbc()),
                                   wheelSurface->X_0_s(polaris, *(polaris.mbc))) << std::endl;
  */

  qpsolver->addTask(polarisPostureTask.get());

  qpsolver->setContacts(drivingContacts);

  LOG_INFO("End reset")
}

void MCDrivingController::resetBasePose()
{
  mc_rbdyn::Robot& polaris = robots().robot(1);
  //Reset freeflyer, compute its position frow wheel and re-set it
  robot().mbc().q[0] = {1., 0., 0., 0., 0., 0., 0.};
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  unsigned int steer_i = polaris.bodyIndexByName("steering_wheel");
  sva::PTransformd X_0_w = polaris.mbc().bodyPosW[steer_i];
  const auto & gripperSurface = robot().surface("RightGripper");
  sva::PTransformd X_0_s = gripperSurface.X_0_s(robot(), robot().mbc());
  //sva::PTransformd X_0_base = X_0_s.inv()*(graspOffset*X_0_w);
  sva::PTransformd X_0_base = X_0_s.inv()*X_0_w;

  const auto quat = Eigen::Quaterniond(X_0_base.rotation()).inverse();
  const Eigen::Vector3d trans(X_0_base.translation());
  std::vector<double> baseQ = {quat.w(), quat.x(), quat.y(), quat.z(),
                               trans.x(), trans.y(), trans.z()};

  robot().mbc().q[0] = baseQ;
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
}

void MCDrivingController::resetWheelTransform()
{
  mc_rbdyn::Robot& polaris = robots().robot(1);
  //Change wheel position
  unsigned int chassis_index = polaris.bodyIndexByName("chassis");
  //Do not take into account potential rotation of steering wheel
  unsigned int joint_index = polaris.jointIndexByName("adjust_steering_wheel");

  const auto & gripperSurface = robot().surface("RightGripper");
  const auto & wheelSurface = polaris.surface("bar_wheel");

  sva::PTransformd X_wheel_s = graspOffset*wheelSurface.X_b_s();

  sva::PTransformd X_0_s = gripperSurface.X_0_s(robot(), robot().mbc());

  sva::PTransformd X_0_chassis = polaris.mbc().bodyPosW[chassis_index];

  sva::PTransformd X_chassis_wheel = (X_wheel_s).inv()*X_0_s*(X_0_chassis).inv();

  polaris.mb().transform(static_cast<int>(joint_index), X_chassis_wheel);
  polaris.mbc().zero(polaris.mb());

  rbd::forwardKinematics(polaris.mb(), polaris.mbc());
  rbd::forwardVelocity(polaris.mb(), polaris.mbc());
}

bool MCDrivingController::changeWheelAngle(double theta)
{
  unsigned int wheel_i = robots().robot(1).jointIndexByName("steering_joint");
  auto p = polarisPostureTask->posture();
  p[wheel_i][0] = theta;
  polarisPostureTask->posture(p);
  return true;
}

bool MCDrivingController::changeGaze(double pan, double tilt)
{
  unsigned int pan_i = robot().jointIndexByName("HEAD_JOINT0");
  unsigned int tilt_i = robot().jointIndexByName("HEAD_JOINT1");
  auto p = postureTask->posture();
  p[pan_i][0] = pan;
  p[tilt_i][0] = tilt;
  postureTask->posture(p);
  return true;
}

bool MCDrivingController::changeAnkleAngle(double theta)
{
  theta_ = (tMax_-tMin_)*theta + tMax_;
  unsigned int ankle_i = robot().jointIndexByName("RLEG_JOINT4");
  auto p = postureTask->posture();
  p[ankle_i][0] = theta_;
  postureTask->posture(p);
  return true;
}

bool MCDrivingController::changeWristAngle(double yaw)
{
  unsigned int wrist_i = robot().jointIndexByName("RARM_JOINT6");
  auto p = postureTask->posture();
  p[wrist_i][0] = yaw;
  postureTask->posture(p);
  return true;
}

bool MCDrivingController::driving_service(double wheel, double ankle, double pan, double tilt)
{
  bool r = true;
  r = r && changeWheelAngle(wheel);
  r = r && changeAnkleAngle(ankle);
  r = r && changeGaze(pan, tilt);
  return r;
}

bool MCDrivingController::read_msg(std::string & msg)
{
  boost::trim(msg);
  if(msg == "lock_head")
  {
    lock_head();
    return true;
  }
  if(msg == "unlock_head")
  {
    unlock_head();
    return true;
  }
  if(msg == "lock_lhand")
  {
    lock_lhand();
    return true;
  }
  if(msg == "unlock_lhand")
  {
    unlock_lhand();
    return true;
  }
  if(msg == "start_logging")
  {
    start_logging();
    return true;
  }
  if(msg == "stop_logging")
  {
    stop_logging();
    return true;
  }
  return false;
}

void MCDrivingController::lock_head()
{
  head_task.reset();
  solver().addTask(&head_task);
}
void MCDrivingController::unlock_head()
{
  solver().removeTask(&head_task);
}
void MCDrivingController::lock_lhand()
{
  lhand_task.reset();
  solver().removeTask(&lhand_task);
}
void MCDrivingController::unlock_lhand()
{
  solver().removeTask(&lhand_task);
}

void MCDrivingController::start_logging()
{
  boost::posix_time::ptime now = boost::posix_time::microsec_clock::local_time();
  bfs::path log_path(getenv("HOME"));
  log_path = log_path / "drc_driving_experiments" / boost::gregorian::to_iso_string(now.date());
  if(!bfs::exists(log_path))
  {
    bfs::create_directories(log_path);
  }
  unsigned int i = 1;
  while(1)
  {
    std::stringstream ss;
    ss << std::setw(3) << std::setfill('0') << i;
    ++i;
    bfs::path test = log_path / ss.str();
    if(!bfs::exists(test))
    {
      bfs::create_directory(test);
      log_path = test;
      break;
    }
  }
  auto new_log = [&log_path](const std::string & log_name, std::ofstream & ofs)
  {
    bfs::path p = log_path / log_name;
    ofs.close();
    ofs.open(p.string());
  };
  new_log("driving-ankle-value.log", log_ankle_);
  new_log("driving-wheel-value.log", log_wheel_);
  new_log("driving-ef-pos.log", log_ef_);
  new_log("driving-acc-sensor.log", log_acc_);
  new_log("driving-rpy-sensor.log", log_rpy_);
  logging_ = true;
}

void MCDrivingController::stop_logging()
{
  logging_ = false;
}

std::vector<std::string> MCDrivingController::supported_robots() const
{
  return {"hrp2_drc"};
}

}
