#include <mc_control/mc_drc_controller.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

/* Called by the RT component to access actual Controllers service */
bool MCDRCGlobalController::change_joint(int jid)
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.change_joint(jid);
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::change_joint(const std::string & jname)
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.change_joint(jname);
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::joint_up()
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.joint_up();
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::joint_down()
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.joint_down();
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::set_joint_pos(const std::string & jname, const double & pos)
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.set_joint_pos(jname, pos);
  }
  else
  {
    return false;
  }
}

bool MCDRCGlobalController::change_ef(const std::string & ef_name)
{
  if(current_ctrl == BODY6D)
  {
    return body6d_controller.change_ef(ef_name);
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::translate_ef(const Eigen::Vector3d & t)
{
  if(current_ctrl == BODY6D)
  {
    return body6d_controller.translate_ef(t);
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::rotate_ef(const Eigen::Matrix3d & m)
{
  if(current_ctrl == BODY6D)
  {
    return body6d_controller.rotate_ef(m);
  }
  else
  {
    return false;
  }
}

bool MCDRCGlobalController::move_com(const Eigen::Vector3d & v)
{
  if(current_ctrl == COM)
  {
    return com_controller.move_com(v);
  }
  else
  {
    return false;
  }
}

bool MCDRCGlobalController::play_next_stance()
{
  if(current_ctrl == SEQ)
  {
    return seq_controller.play_next_stance();
  }
  else
  {
    return false;
  }
}

}
