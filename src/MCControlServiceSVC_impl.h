#ifndef _H_MCCONTROLSERVICESVCIMPL_H_
#define _H_MCCONTROLSERVICESVCIMPL_H_

#include "MCControlService.hh"

class MCControl;

namespace OpenHRP
{
  class MCControlServiceSVC_impl:
    public virtual POA_OpenHRP::MCControlService,
    public virtual PortableServer::RefCountServantBase
  {
    public:
      MCControlServiceSVC_impl(MCControl * plugin);
      virtual ~MCControlServiceSVC_impl();

      /* General services to switch between controllers */
      virtual CORBA::Boolean EnablePostureController();

      /* Joint services */
      virtual CORBA::Boolean change_joint(const char* jname);
      virtual CORBA::Boolean joint_up();
      virtual CORBA::Boolean joint_down();
      virtual CORBA::Boolean set_joint_pos(const char* jname, ::CORBA::Double v);
    private:
      MCControl * m_plugin;
  };
}

#endif
