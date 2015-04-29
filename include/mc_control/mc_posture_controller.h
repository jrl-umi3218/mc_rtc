#ifndef _H_MCPOSTURECONTROLLER_H_
#define _H_MCPOSTURECONTROLLER_H_

#include <mc_control/mc_controller.h>

/* FIXME For now, everything is in there, split it would be good */

namespace mc_control
{

struct MCPostureController : public MCController
{
public:
  /* Common stuff */
  MCPostureController();

  /* Specific to posture controller */
  bool change_joint(int jid);
  bool change_joint(const std::string & jname);

  bool joint_up();
  bool joint_down();

  bool set_joint_pos(const std::string & jname, const double & pos);
public:
  /* Specific to posture controller */
  int current_joint;
private:
  void add_joint_pos(const double & v);
};

}

#endif
