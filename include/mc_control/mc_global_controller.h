#pragma once

#include <mc_control/mc_controller.h>

#include <mc_control/api.h>

#include <mc_rbdyn/RobotModule.h>

#include <mc_rtc/config.h>
#include <mc_rtc/loader.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <array>
#include <fstream>
#include <thread>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCGlobalController
{
public:
  MCGlobalController(const std::string & conf = mc_rtc::CONF_PATH);
  virtual ~MCGlobalController();

  void init(const std::vector<double> & initq);

  void init(const std::vector<double> & initq, const std::array<double, 7> & initAttitude);

  void setSensorPosition(const Eigen::Vector3d & pos);

  void setSensorOrientation(const Eigen::Vector3d & ori);

  void setSensorLinearVelocity(const Eigen::Vector3d& vel);

  void setSensorAngularVelocity(const Eigen::Vector3d& vel);

  void setSensorAcceleration(const Eigen::Vector3d& acc);

  void setEncoderValues(const std::vector<double> & eValues);

  void setJointTorques(const std::vector<double> & tValues);

  void setWrenches(const std::map<std::string, sva::ForceVecd> & wrenches);

  void setActualGripperQ(const std::map<std::string, std::vector<double>> & grippersQ);

  bool run();

  const QPResultMsg & send(const double & t);

  const mc_rbdyn::Robot & robot();

  double timestep();

  const std::vector<std::string> & ref_joint_order();

  /* Called by the RT component to switch between controllers */
  bool EnableController(const std::string & name);

  /* Gripper controls */
  std::map<std::string, std::vector<double>> gripperQ();
  std::map<std::string, std::vector<std::string>> gripperJoints();
  std::map<std::string, std::vector<std::string>> gripperActiveJoints();
  void setGripperCurrentQ(const std::map<std::string, std::vector<double>> & gripperQs);
  void setGripperTargetQ(const std::string & name, const std::vector<double> & q);
  void setGripperOpenPercent(double pOpen);
  void setGripperOpenPercent(const std::string & name, double pOpen);

  /* Control the posture if provided by the controller */
  bool set_joint_pos(const std::string & jname, const double & pos);
  bool get_joint_pos(const std::string & jname, double & pos);

  /* Control the EFs if provided by the current controller */
  bool change_ef(const std::string & ef_name);
  bool translate_ef(const Eigen::Vector3d & t);
  bool rotate_ef(const Eigen::Matrix3d & m);

  /* Control the CoM if provided by the current controller */
  bool move_com(const Eigen::Vector3d & v);

  /* Start playing the next stance if in seq controller */
  bool play_next_stance();

  /* Return to halfsit pose at the end of an experiment */
  bool GoToHalfSitPose_service();
  bool GoToHalfSitPose();

  /*Change wheel or ankle angle if in driving controller*/
  bool change_wheel_angle(double theta);
  bool change_ankle_angle(double theta);
  bool change_gaze(double pan, double tilt);
  bool change_wrist_angle(double yaw);
  bool driving_service(double w, double a, double p, double t);
  /* Generic message passing service */
  bool send_msg(const std::string & msg);
  bool send_recv_msg(const std::string & msg, std::string & out);
private:
  void publish_thread();

public:
  bool running;
private:
  struct Configuration
  {
    Configuration(const std::string & path);

    inline bool enabled(const std::string & ctrl);

    std::vector<std::string> robot_module_paths;
    std::shared_ptr<mc_rbdyn::RobotModule> main_robot_module;

    std::vector<std::string> controller_module_paths;
    std::vector<std::string> enabled_controllers;
    std::string initial_controller;
    double timestep;

    bool publish_control_state;
    bool publish_real_state;
    double publish_timestep;

    bool enable_log;
    bfs::path log_directory;
    std::string log_template;

    Json::Value v;
  };
private:
  Configuration config;
  std::map<std::string, std::shared_ptr<mc_control::MCController>> controllers;
  std::string current_ctrl;
  std::string next_ctrl;
  MCController * controller;
  MCController * next_controller;
  std::unique_ptr<mc_rtc::ObjectLoader<MCController>> controller_loader;

  bool publish_th_running = true;
  std::thread publish_th;
  mc_rbdyn::Robots real_robots;

  std::ofstream log_;
  double log_iter_;
  void log_header();
  void log_data();
};

}
