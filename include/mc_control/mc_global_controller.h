#pragma once

#include <mc_control/mc_posture_controller.h>
#include <mc_control/mc_body6d_controller.h>
#include <mc_control/mc_com_controller.h>
#include <mc_control/mc_seq_controller.h>
#include <mc_control/mc_driving_controller.h>
#include <mc_control/mc_egress_controller.h>
#include <mc_control/mc_egress_mrqp_controller.h>

/* FIXME For now, everything is in there, split it would be good */

namespace mc_control
{

struct MCGlobalController
{
public:
  MCGlobalController();

  void init(const std::vector<double> & initq);

  void setSensorOrientation(const Eigen::Vector3d & ori);

  void setEncoderValues(const std::vector<double> & eValues);

  void setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches);

  void setActualGripperQ(double rQ, double lQ);

  bool run();

  const QPResultMsg & send(const double & t);

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
  bool change_joint(int jid);
  bool change_joint(const std::string & jname);
  bool joint_up();
  bool joint_down();
  bool set_joint_pos(const std::string & jname, const double & pos);

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

public:
  bool running;
private:
  MCPostureController posture_controller;
  MCBody6dController body6d_controller;
  MCCoMController com_controller;
  MCSeqController seq_controller;
  MCDrivingController driving_controller;
  MCEgressController egress_controller;
  MCEgressMRQPController egress_mrqp_controller;
  enum CurrentController
  {
    POSTURE = 1,
    BODY6D  = 2,
    COM = 3,
    SEQ = 4,
    DRIVING = 5,
    EGRESS = 6,
    EGRESS_MRQP = 7
  };
  CurrentController current_ctrl;
  CurrentController next_ctrl;
  MCVirtualController * controller;
  MCVirtualController * next_controller;
};

}
