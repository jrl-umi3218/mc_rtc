#ifndef _H_MCDRCCONTROLLER_H_
#define _H_MCDRCCONTROLLER_H_

#include <mc_control/mc_posture_controller.h>
#include <mc_control/mc_body6d_controller.h>
#include <mc_control/mc_com_controller.h>
#include <mc_control/mc_seq_controller.h>

/* FIXME For now, everything is in there, split it would be good */

namespace mc_control
{

struct MCDRCGlobalController
{
public:
  MCDRCGlobalController();

  void init(const std::vector<double> & initq);

  bool run();

  const mc_control::QPResultMsg & send(const double & t);

  const mc_solver::QPSolver & qpsolver() const;
  /* Called by the RT component to switch between controllers */
  bool EnablePostureController();
  bool EnableBody6dController();
  bool EnableCoMController();
  bool EnableSeqController();

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
public:
  bool running;
private:
  MCPostureController posture_controller;
  MCBody6dController body6d_controller;
  MCCoMController com_controller;
  MCSeqController seq_controller;
  enum CurrentController
  {
    POSTURE = 1,
    BODY6D  = 2,
    COM = 3,
    SEQ = 4
  };
  CurrentController current_ctrl;
  CurrentController next_ctrl;
  MCController * controller;
  MCController * next_controller;
};

}

#endif
