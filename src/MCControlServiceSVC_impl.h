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

      virtual CORBA::Boolean EnableBody6dController();

      virtual CORBA::Boolean EnableCoMController();

      virtual CORBA::Boolean EnableSeqController();

      virtual CORBA::Boolean EnableDrivingController();

      /* Grippers (always available) */
      virtual CORBA::Boolean open_grippers();

      virtual CORBA::Boolean close_grippers();

      virtual CORBA::Boolean set_gripper(CORBA::Boolean lgripper, CORBA::Double v);

      /* Joint services */
      virtual CORBA::Boolean change_joint(const char* jname);
      virtual CORBA::Boolean joint_up();
      virtual CORBA::Boolean joint_down();
      virtual CORBA::Boolean set_joint_pos(const char* jname, CORBA::Double v);
      /* End effector service */
      virtual CORBA::Boolean change_ef(const char * body);
      virtual CORBA::Boolean translate_ef(CORBA::Double x, CORBA::Double y, CORBA::Double z);
      virtual CORBA::Boolean rotate_ef(CORBA::Double r, CORBA::Double p, CORBA::Double y);
      /* CoM services */
      virtual CORBA::Boolean move_com(CORBA::Double x, CORBA::Double y, CORBA::Double z);
      /* Seq controller */
      virtual CORBA::Boolean play_next_stance();

      virtual CORBA::Boolean change_wheel_angle(CORBA::Double theta);
      virtual CORBA::Boolean change_ankle_angle(CORBA::Double theta);
      virtual CORBA::Boolean change_gaze(CORBA::Double pan, CORBA::Double tilt);
      virtual CORBA::Boolean change_wrist_angle(CORBA::Double yaw);
    private:
      MCControl * m_plugin;
  };
}

#endif
