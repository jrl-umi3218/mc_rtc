#ifndef _H_MCDRCCONTROLLER_H_
#define _H_MCDRCCONTROLLER_H_

#include <mc_control/mc_posture_controller.h>
#include <mc_control/mc_body6d_controller.h>

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

  /* Called by the RT component to access PostureController service */
  bool change_joint(int jid);
  bool change_joint(const std::string & jname);
  bool joint_up();
  bool joint_down();
  bool set_joint_pos(const std::string & jname, const double & pos);

  /* Called by the RT component to access Body6dController service */
  bool change_ef(const std::string & ef_name);
  bool translate_ef(const Eigen::Vector3d & t);
  bool rotate_ef(const Eigen::Matrix3d & m);
public:
  bool running;
private:
  MCPostureController posture_controller;
  MCBody6dController body6d_controller;
  enum CurrentController
  {
    POSTURE = 1,
    BODY6D  = 2
  };
  CurrentController current_ctrl;
  CurrentController next_ctrl;
  MCController * controller;
  MCController * next_controller;
};

}

#endif
