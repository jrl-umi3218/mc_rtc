#include "MCControlServiceSVC_impl.h"

/* Trick to disable compiler warning on this part of the code */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#include "MCControlServiceSK.cc"
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

}
