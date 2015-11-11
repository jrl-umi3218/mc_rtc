#pragma once

#include <mc_control/mc_posture_controller.h>
#include <mc_control/mc_body6d_controller.h>
#include <mc_control/mc_com_controller.h>
#include <mc_control/mc_seq_controller.h>
#include <mc_control/mc_driving_controller.h>
#include <mc_control/mc_egress_controller.h>
#include <mc_control/mc_egress_mrqp_controller.h>
#include <mc_control/mc_bci_self_interact_controller.h>

namespace mc_control
{

struct MCGlobalController
{
public:
  MCGlobalController();

  void init(const std::vector<double> & initq);

  void setSensorOrientation(const Eigen::Vector3d & ori);

  void setSensorAcceleration(const Eigen::Vector3d& acc);

  void setEncoderValues(const std::vector<double> & eValues);

  void setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches);

  void setActualGripperQ(double rQ, double lQ);

  bool run();

  const QPResultMsg & send(const double & t);

  const mc_rbdyn::Robot & robot();

  /* Called by the RT component to switch between controllers */
  bool EnablePostureController();
  bool EnableBody6dController();
  bool EnableCoMController();
  bool EnableSeqController();
  bool EnableDrivingController();
  bool EnableEgressController();

  /* Gripper controls */
  const std::vector<double> & gripperQ(bool lgripper);
  void setGripperCurrentQ(double lQ, double rQ);
  void setGripperTargetQ(double lQ, double rQ);
  void setLGripperTargetQ(double lQ);
  void setRGripperTargetQ(double rQ);
  void setGripperOpenPercent(double lQ, double rQ);

  /* Control the posture if provided by the controller */
  bool joint_up(const std::string & jname);
  bool joint_down(const std::string & jname);
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

  /*Change wheel or ankle angle if in driving controller*/
  bool change_wheel_angle(double theta);
  bool change_ankle_angle(double theta);
  bool change_gaze(double pan, double tilt);
  bool change_wrist_angle(double yaw);
  bool driving_service(double w, double a, double p, double t);
  /* Generic message passing service */
  bool send_msg(const std::string & msg);
  bool send_recv_msg(const std::string & msg, std::string & out);
public:
  bool running;
private:
  struct Configuration
  {
    Configuration(const std::string & path);

    inline bool enabled(const std::string & ctrl);

    std::vector<std::string> enabled_controllers;
    std::string initial_controller;
    std::string seq_env_path;
    std::string seq_env_name;
    std::string seq_env_module;
    std::string seq_plan;
    bool seq_step_by_step;
    bool seq_use_real_sensors;
    unsigned int seq_start_stance;
  };
private:
  Configuration config;
  std::shared_ptr<MCPostureController> posture_controller;
  std::shared_ptr<MCBody6dController> body6d_controller;
  std::shared_ptr<MCCoMController> com_controller;
  std::shared_ptr<MCSeqController> seq_controller;
  std::shared_ptr<MCDrivingController> driving_controller;
  std::shared_ptr<MCEgressController> egress_controller;
  std::shared_ptr<MCEgressMRQPController> egress_mrqp_controller;
  std::shared_ptr<MCBCISelfInteractController> bci_self_interact_controller;
  enum CurrentController
  {
    POSTURE = 1,
    BODY6D  = 2,
    COM = 3,
    SEQ = 4,
    DRIVING = 5,
    EGRESS = 6,
    EGRESS_MRQP = 7,
    BCISELFINTERACT = 8,
    NONE = 42
  };
  CurrentController current_ctrl;
  CurrentController next_ctrl;
  MCVirtualController * controller;
  MCVirtualController * next_controller;
  bool EnableNextController(const std::string & name, const CurrentController & index, const std::shared_ptr<MCVirtualController> & ctrl);
};

}
