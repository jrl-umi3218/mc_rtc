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
    m_qOutOut("qOut", m_qOut),
    m_MCControlServicePortPort("MCControlServicePort"),
    m_service0(this)

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
  bindParameter("timeStep", m_timeStep, "0.005");
  bindParameter("is_enabled", controller.running, "0");

  // </rtc-template>
  std::cout << "MCControl::onInitialize() finished" << std::endl;
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
      double t = tm.sec*1e9 + tm.nsec;
      if(controller.run())
      {
        const mc_control::QPResultMsg & res = controller.send(t);
        m_qOut.data.length(m_qIn.data.length());
        for(unsigned int i = 7; i < res.q.size() + 1; ++i)
        {
          if(i < 30)
          {
            m_qOut.data[i-7] = res.q[i];
          }
          else if(i == 30)
          {
            m_qOut.data[i-7] = qIn[i];
          }
          else
          {
            m_qOut.data[i-7] = res.q[i-1];
          }
        }
        for(unsigned int i = 37; i < m_qIn.data.length(); ++i)
        {
          m_qOut.data[i] = qIn[i];
        }
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
    }
    else
    {
      m_qOut = m_qIn;
    }
    m_qOut.tm = tm;
    m_qOutOut.write();
  }
  return RTC::RTC_OK;
}

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

