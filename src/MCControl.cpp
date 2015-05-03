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
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wdelete-incomplete"
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#include "MCControl.h"

#include <fstream>

static std::ofstream ofs("/tmp/mc-control.log");

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
    "conf.default.timeStep", "0.005",
    "conf.default.is_enabled", "0",
    ""
  };
// </rtc-template>

MCControl::MCControl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_qInIn("qIn", m_qIn),
    m_pInIn("pIn", m_pIn),
    m_rpyInIn("rpyIn", m_rpyIn),
    m_qOutOut("qOut", m_qOut),
    m_pOutOut("pOut", m_pOut),
    m_rpyOutOut("rpyOut", m_rpyOut),
    m_MCControlServicePortPort("MCControlServicePort"),
    m_service0(this),
    max_t(0),
    init(false)

    // </rtc-template>
{
  std::cout << "MCControl::MCControl" << std::endl;
}

MCControl::~MCControl()
{
}


RTC::ReturnCode_t MCControl::onInitialize()
{
  std::cout << "MCControl::onInitialize() starting" << std::endl;
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qIn", m_qInIn);
  addInPort("pIn", m_pInIn);
  addInPort("rpyIn", m_rpyInIn);

  // Set OutPort buffer
  addOutPort("qOut", m_qOutOut);
  addOutPort("pOut", m_pOutOut);
  addOutPort("rpyOut", m_rpyOutOut);

  // Set service provider to Ports
  m_MCControlServicePortPort.registerProvider("service0", "MCControlService", m_service0);

  // Set service consumers to Ports

  // Set CORBA Service Ports
  addPort(m_MCControlServicePortPort);

  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("timeStep", m_timeStep, "0.005");
  bindParameter("is_enabled", controller.running, "0");

  // </rtc-template>
  std::cout << "MCControl::onInitialize() finished" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCControl::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MCControl::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t MCControl::onExecute(RTC::UniqueId ec_id)
{
  if(m_qInIn.isNew())
  {
    m_qInIn.read();
    qIn.resize(m_qIn.data.length());
    for(unsigned int i = 0; i < m_qIn.data.length(); ++i)
    {
      qIn[i] = m_qIn.data[i];
    }
    coil::TimeValue coiltm(coil::gettimeofday());
    RTC::Time tm;
    tm.sec = coiltm.sec();
    tm.nsec = coiltm.usec()*1e3;
    if(controller.running)
    {
      if(!init)
      {
        controller.init(qIn);
        init = true;
      }
      double t = tm.sec*1e9 + tm.nsec;
      if(controller.run())
      {
        const mc_control::QPResultMsg & res = controller.send(t);
        m_qOut.data.length(m_qIn.data.length());
        for(unsigned int i = 0; i < 23; ++i)
        {
          m_qOut.data[i] = res.q[i+7];
        }
        m_qOut.data[23] = m_qIn.data[23];
        for(unsigned int i = 24; i < 31; ++i)
        {
          m_qOut.data[i] = res.q[i+12];
        }
        for(unsigned int i = 31; i < 42; ++i)
        {
          m_qOut.data[i] = m_qIn.data[i];
        }
        /* FIXME Correction RPY convention here? */
        Eigen::Vector3d rpy = Eigen::Quaterniond(res.q[0], res.q[1], res.q[2], res.q[3]).toRotationMatrix().eulerAngles(0, 1, 2);
        m_rpyOut.data.r = rpy[0];
        m_rpyOut.data.p = rpy[1];
        m_rpyOut.data.y = rpy[2];
        m_pOut.data.x = res.q[4];
        m_pOut.data.y = res.q[5];
        m_pOut.data.z = res.q[6];
        //ofs << "qIn" << std::endl;
        //for(size_t i = 0; i < m_qIn.data.length(); ++i)
        //{
        //  ofs << "qIn[" << i << "] = " << m_qIn.data[i] << std::endl;
        //}
        //ofs << "res.q" << std::endl;
        //for(size_t i = 0; i < res.q.size(); ++i)
        //{
        //  ofs << "res.q[" << i << "] = " << res.q[i] << std::endl;
        //}
        //ofs << "qOut" << std::endl;
        //for(size_t i = 0; i < m_qOut.data.length(); ++i)
        //{
        //  ofs << "qOut[" << i << "] = " << m_qOut.data[i] << std::endl;
        //}
      }
      m_qOut.tm = tm;
      m_rpyOut.tm = tm;
      m_pOut.tm = tm;
      m_qOutOut.write();
      m_pOutOut.write();
      m_rpyOutOut.write();
    }
    else
    {
      init = false;
      m_qOut = m_qIn;
      /* Still run controller.run() in order to handle some service calls */
      controller.run();
    }
    coil::TimeValue coiltmout(coil::gettimeofday());
    if((coiltmout.sec() - coiltm.sec())*1e6 + coiltmout.usec() - coiltm.usec() > max_t)
    {
      max_t = (coiltmout.sec() - coiltm.sec())*1e6 + coiltmout.usec() - coiltm.usec();
    }
    std::cout << "\rMax time spent in controller: " << max_t << " ms" << std::flush;
  }
  return RTC::RTC_OK;
}

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

