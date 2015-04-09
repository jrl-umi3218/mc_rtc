// -*- C++ -*-
/*!
 * @file  MCControl.cpp * @brief Core component for MC control * $Date$ 
 *
 * $Id$ 
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wdelete-incomplete"
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#include "MCControl.h"

// Module specification
// <rtc-template block="module_spec">
static const char* mccontrol_spec[] =
  {
    "implementation_id", "MCControl",
    "type_name",         "MCControl",
    "description",       "Core component for MC control",
    "version",           "0.1",
    "vendor",            "CNRS",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.robot_urdf_url", "/home/gergondet/catkin_ws/src/hrp2_drc/hrp2_drc_description/urdf/hrp2drc.urdf",
    "conf.default.stances_file", "/home/gergondet/devel-src/mcp/mcp_ws/src/mc_ros/mc_control_scenario/stances/hrp2_drc/drc_multi_robot_frame_1_87b.seq",
    "conf.default.conf_file", "/home/gergondet/devel-src/mcp/mcp_ws/src/mc_ros/mc_control_scenario/config/classic/hrp2_drc/drc_test_conf.py",
    ""
  };
// </rtc-template>

MCControl::MCControl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_qInIn("qIn", m_qIn),
    m_qOutOut("qOut", m_qOut),
    m_MCControlServicePortPort("MCControlServicePort")

    // </rtc-template>
{
}

MCControl::~MCControl()
{
}


RTC::ReturnCode_t MCControl::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qIn", m_qInIn);

  // Set OutPort buffer
  addOutPort("qOut", m_qOutOut);

  // Set service provider to Ports
  m_MCControlServicePortPort.registerProvider("service0", "MCControlService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_MCControlServicePortPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("robot_urdf_url", m_robot_urdf_url, "/home/gergondet/catkin_ws/src/hrp2_drc/hrp2_drc_description/urdf/hrp2drc.urdf");
  bindParameter("stances_file", m_stances_file, "/home/gergondet/devel-src/mcp/mcp_ws/src/mc_ros/mc_control_scenario/stances/hrp2_drc/drc_multi_robot_frame_1_87b.seq");
  bindParameter("conf_file", m_conf_file, "/home/gergondet/devel-src/mcp/mcp_ws/src/mc_ros/mc_control_scenario/config/classic/hrp2_drc/drc_test_conf.py");
  bindParameter("timeStep", m_timeStep, "0.002");

  // </rtc-template>
  return RTC::RTC_OK;
}


/*
RTC::ReturnCode_t MCControl::onFinalize()
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/
/*
RTC::ReturnCode_t MCControl::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


extern "C"
{
 
  void MCControlInit(RTC::Manager* manager)
  {
    coil::Properties profile(mccontrol_spec);
    manager->registerFactory(profile,
                             RTC::Create<MCControl>,
                             RTC::Delete<MCControl>);
  }
  
};


#pragma GCC diagnostic pop

