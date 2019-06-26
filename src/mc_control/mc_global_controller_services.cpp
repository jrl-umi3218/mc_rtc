/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/ros.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

/* Called by the RT component to access actual controller_s service */
bool MCGlobalController::set_joint_pos(const std::string & jname, const double & pos)
{
  if(controller_)
  {
    return controller_->set_joint_pos(jname, pos);
  }
  else
  {
    return false;
  }
}

bool MCGlobalController::get_joint_pos(const std::string & jname, double & v)
{
  if(controller_)
  {
    return controller_->get_joint_pos(jname, v);
  }
  else
  {
    return false;
  }
}

bool MCGlobalController::change_ef(const std::string & ef_name)
{
  if(controller_)
  {
    return controller_->change_ef(ef_name);
  }
  else
  {
    return false;
  }
}
bool MCGlobalController::translate_ef(const Eigen::Vector3d & t)
{
  if(controller_)
  {
    return controller_->move_ef(t, Eigen::Matrix3d::Identity());
  }
  else
  {
    return false;
  }
}
bool MCGlobalController::rotate_ef(const Eigen::Matrix3d & m)
{
  if(controller_)
  {
    return controller_->move_ef(Eigen::Vector3d(0, 0, 0), m);
  }
  else
  {
    return false;
  }
}

bool MCGlobalController::move_com(const Eigen::Vector3d & v)
{
  if(controller_)
  {
    return controller_->move_com(v);
  }
  else
  {
    return false;
  }
}

bool MCGlobalController::play_next_stance()
{
  if(controller_)
  {
    return controller_->play_next_stance();
  }
  else
  {
    return false;
  }
}

bool MCGlobalController::driving_service(double w, double a, double p, double t)
{
  if(controller_)
  {
    return controller_->driving_service(w, a, p, t);
  }
  else
  {
    return false;
  }
}

bool MCGlobalController::GoToHalfSitPose_service()
{
  if(controller_)
  {
    return GoToHalfSitPose();
  }
  else
  {
    return false;
  }
}

bool MCGlobalController::send_msg(const std::string & msg)
{
  if(controller_)
  {
    return controller_->read_msg(const_cast<std::string &>(msg));
  }
  return false;
}

bool MCGlobalController::send_recv_msg(const std::string & msg, std::string & out)
{
  if(controller_)
  {
    return controller_->read_write_msg(const_cast<std::string &>(msg), out);
  }
  return false;
}

} // namespace mc_control
