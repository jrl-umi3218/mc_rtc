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

namespace OpenHRP
{

MCControlServiceSVC_impl::MCControlServiceSVC_impl(MCControl * plugin)
: m_plugin(plugin)
{
}

MCControlServiceSVC_impl::~MCControlServiceSVC_impl()
{
}

/*FIXME Not implemented */
CORBA::Boolean MCControlServiceSVC_impl::EnablePostureController()
{
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


}
