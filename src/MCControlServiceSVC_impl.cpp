#include "MCControlServiceSVC_impl.h"

/* Trick to disable compiler warning on this part of the code */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#pragma GCC diagnostic pop

#include <iostream>

#include "MCControl.h"

#include <mc_rbdyn/surface.h>

namespace OpenHRP
{

MCControlServiceSVC_impl::MCControlServiceSVC_impl(MCControl * plugin)
: m_plugin(plugin)
{
}

MCControlServiceSVC_impl::~MCControlServiceSVC_impl()
{
}

CORBA::Boolean MCControlServiceSVC_impl::EnablePostureController()
{
  return m_plugin->controller.EnablePostureController();
}

CORBA::Boolean MCControlServiceSVC_impl::EnableBody6dController()
{
  return m_plugin->controller.EnableBody6dController();
}

CORBA::Boolean MCControlServiceSVC_impl::EnableCoMController()
{
  return m_plugin->controller.EnableCoMController();
}

CORBA::Boolean MCControlServiceSVC_impl::EnableSeqController()
{
  return m_plugin->controller.EnableSeqController();
}

CORBA::Boolean MCControlServiceSVC_impl::EnableDrivingController()
{
  return m_plugin->controller.EnableDrivingController();
}

CORBA::Boolean MCControlServiceSVC_impl::open_grippers()
{
  m_plugin->controller.setGripperOpenPercent(1, 1);
  return true;
}

CORBA::Boolean MCControlServiceSVC_impl::close_grippers()
{
  m_plugin->controller.setGripperOpenPercent(0, 0);
  return true;
}

CORBA::Boolean MCControlServiceSVC_impl::set_gripper(CORBA::Boolean lgripper, CORBA::Double v)
{
  if(lgripper)
  {
    m_plugin->controller.setLGripperTargetQ(v);
  }
  else
  {
    m_plugin->controller.setRGripperTargetQ(v);
  }
  return true;
}

CORBA::Boolean MCControlServiceSVC_impl::change_joint(const char* jname)
{
  return m_plugin->controller.change_joint(jname);
}

CORBA::Boolean MCControlServiceSVC_impl::joint_up()
{
  return m_plugin->controller.joint_up();
}

CORBA::Boolean MCControlServiceSVC_impl::joint_down()
{
  return m_plugin->controller.joint_down();
}

CORBA::Boolean MCControlServiceSVC_impl::set_joint_pos(const char* jname, ::CORBA::Double v)
{
  return m_plugin->controller.set_joint_pos(jname, v);
}

CORBA::Boolean MCControlServiceSVC_impl::change_ef(const char * body)
{
  return m_plugin->controller.change_ef(body);
}

CORBA::Boolean MCControlServiceSVC_impl::translate_ef(CORBA::Double x, CORBA::Double y, CORBA::Double z)
{
  return m_plugin->controller.translate_ef(Eigen::Vector3d(x, y, z));
}

CORBA::Boolean MCControlServiceSVC_impl::rotate_ef(CORBA::Double r, CORBA::Double p, CORBA::Double y)
{
  Eigen::Matrix3d rpy = mc_rbdyn::rpyToMat(r, p, y);
  return m_plugin->controller.rotate_ef(rpy);
}

CORBA::Boolean MCControlServiceSVC_impl::move_com(CORBA::Double x, CORBA::Double y, CORBA::Double z)
{
  return m_plugin->controller.move_com(Eigen::Vector3d(x, y, z));
}

CORBA::Boolean MCControlServiceSVC_impl::play_next_stance()
{
  return m_plugin->controller.play_next_stance();
}

CORBA::Boolean MCControlServiceSVC_impl::change_wheel_angle(CORBA::Double theta)
{
  return m_plugin->controller.change_wheel_angle(theta);
}

CORBA::Boolean MCControlServiceSVC_impl::change_ankle_angle(CORBA::Double theta)
{
  return m_plugin->controller.change_ankle_angle(theta);
}

CORBA::Boolean MCControlServiceSVC_impl::change_gaze(CORBA::Double pan, CORBA::Double tilt)
{
  return m_plugin->controller.change_gaze(pan, tilt);
}

CORBA::Boolean MCControlServiceSVC_impl::change_wrist_angle(CORBA::Double yaw)
{
  return m_plugin->controller.change_wrist_angle(yaw);
}

CORBA::Boolean MCControlServiceSVC_impl::driving_service(CORBA::Double w, CORBA::Double a, CORBA::Double p, CORBA::Double t)
{
  return m_plugin->controller.driving_service(w, a, p, t);
}

}
