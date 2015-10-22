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
      virtual CORBA::Boolean EnablePostureController() override;

      virtual CORBA::Boolean EnableBody6dController() override;

      virtual CORBA::Boolean EnableCoMController() override;

      virtual CORBA::Boolean EnableSeqController() override;

      virtual CORBA::Boolean EnableDrivingController() override;

      /* Grippers (always available) */
      virtual CORBA::Boolean open_grippers() override;

      virtual CORBA::Boolean close_grippers() override;

      virtual CORBA::Boolean set_gripper(CORBA::Boolean lgripper, CORBA::Double v) override;

      /* Joint services */
      virtual CORBA::Boolean joint_up(const char * jname) override;
      virtual CORBA::Boolean joint_down(const char * jname) override;
      virtual CORBA::Boolean set_joint_pos(const char* jname, CORBA::Double v) override;
      virtual CORBA::Boolean get_joint_pos(const char * jname, CORBA::Double & v) override;
      /* End effector service */
      virtual CORBA::Boolean change_ef(const char * body) override;
      virtual CORBA::Boolean translate_ef(CORBA::Double x, CORBA::Double y, CORBA::Double z) override;
      virtual CORBA::Boolean rotate_ef(CORBA::Double r, CORBA::Double p, CORBA::Double y) override;
      /* CoM services */
      virtual CORBA::Boolean move_com(CORBA::Double x, CORBA::Double y, CORBA::Double z) override;
      /* Seq controller */
      virtual CORBA::Boolean play_next_stance() override;
      /* Driving services */
      virtual CORBA::Boolean change_wheel_angle(CORBA::Double theta) override;
      virtual CORBA::Boolean change_ankle_angle(CORBA::Double theta) override;
      virtual CORBA::Boolean change_gaze(CORBA::Double pan, CORBA::Double tilt) override;
      virtual CORBA::Boolean change_wrist_angle(CORBA::Double yaw) override;
      virtual CORBA::Boolean driving_service(CORBA::Double w, CORBA::Double a, CORBA::Double p, CORBA::Double t) override;

      /* Generic method, allows any controller to implement simple service calls */
      virtual CORBA::Boolean send_msg(const char * msg) override;
    private:
      MCControl * m_plugin;
  };
}

#endif
