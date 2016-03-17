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

namespace OpenHRP
{

MCControlServiceSVC_impl::MCControlServiceSVC_impl(MCControl * plugin)
: m_plugin(plugin)
{
}

MCControlServiceSVC_impl::~MCControlServiceSVC_impl()
{
}

CORBA::Boolean MCControlServiceSVC_impl::EnableController(const char * name)
{
  return m_plugin->controller.EnableController(name);
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

CORBA::Boolean MCControlServiceSVC_impl::set_joint_pos(const char* jname, ::CORBA::Double v)
{
  return m_plugin->controller.set_joint_pos(jname, v);
}

CORBA::Boolean MCControlServiceSVC_impl::get_joint_pos(const char* jname, ::CORBA::Double & v)
{
  return m_plugin->controller.get_joint_pos(jname, v);
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
  //Eigen::Matrix3d rpy = mc_rbdyn::rpyToMat(r, p, y);
  auto rpy = sva::RotX(r)*sva::RotY(p)*sva::RotZ(y);
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

CORBA::Boolean MCControlServiceSVC_impl::driving_service(CORBA::Double w, CORBA::Double a, CORBA::Double p, CORBA::Double t)
{
  return m_plugin->controller.driving_service(w, a, p, t);
}

CORBA::Boolean MCControlServiceSVC_impl::send_msg(const char * msg)
{
  return m_plugin->controller.send_msg(msg);
}

CORBA::Boolean MCControlServiceSVC_impl::send_recv_msg(const char * msg, ::CORBA::String_out out)
{
  std::string out_str;
  bool b = m_plugin->controller.send_recv_msg(msg, out_str);
  out = CORBA::string_dup(out_str.c_str());
  return b;
}

}
