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

void MCControlServiceSVC_impl::place_holder()
{
  m_plugin->controller.change_joint("RARM_JOINT0");
  m_plugin->controller.joint_up();
}

}
