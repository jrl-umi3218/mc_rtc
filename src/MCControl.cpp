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
    m_wrenchesNames({"RightFootForceSensor", "LeftFootForceSensor", "RightHandForceSensor", "LeftHandForceSensor"}),
    m_qOutOut("qOut", m_qOut),
    m_pOutOut("pOut", m_pOut),
    m_rpyOutOut("rpyOut", m_rpyOut),
    m_MCControlServicePortPort("MCControlServicePort"),
    m_service0(this),
    max_t(0),
    init(false)

    // </rtc-template>
{
  m_wrenchesIn.resize(0);
  m_wrenchesInIn.resize(0);
  for(size_t i = 0; i < m_wrenchesNames.size(); ++i)
  {
    m_wrenchesIn.push_back(new TimedDoubleSeq());
    m_wrenchesInIn.push_back(new InPort<TimedDoubleSeq>(m_wrenchesNames[i].c_str(), *(m_wrenchesIn[i])));
    m_wrenches.push_back(std::pair<Eigen::Vector3d, Eigen::Vector3d>(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)));
  }
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
  for(size_t i = 0; i < m_wrenchesNames.size(); ++i)
  {
    addInPort(m_wrenchesNames[i].c_str(), *(m_wrenchesInIn[i]));
  }

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
  for(size_t i = 0; i < m_wrenchesInIn.size(); ++i)
  {
    if(m_wrenchesInIn[i]->isNew())
    {
      m_wrenchesInIn[i]->read();
      if(m_wrenchesIn[i]->data.length() == 6)
      {
        m_wrenches[i].first = Eigen::Vector3d(m_wrenchesIn[i]->data[0], m_wrenchesIn[i]->data[1], m_wrenchesIn[i]->data[2]);
        m_wrenches[i].second = Eigen::Vector3d(m_wrenchesIn[i]->data[3], m_wrenchesIn[i]->data[4], m_wrenchesIn[i]->data[5]);
      }
    }
  }
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
      controller.setWrenches(m_wrenches);
      if(controller.run())
      {
        const mc_control::QPResultMsg & res = controller.send(t);
        const std::vector<double> & lgQ = controller.gripperQ(true);
        const std::vector<double> & rgQ = controller.gripperQ(false);
        controller.setActualGripperQ(m_qIn.data[23], m_qIn.data[31]);
        m_qOut.data.length(m_qIn.data.length());
        for(unsigned int i = 0; i < 23; ++i)
        {
          m_qOut.data[i] = res.q[i+7];
        }
        m_qOut.data[23] = rgQ[0];
        for(unsigned int i = 24; i < 31; ++i)
        {
          m_qOut.data[i] = res.q[i+12];
        }
        m_qOut.data[31] = lgQ[0];
        for(unsigned int i = 32; i < 37; ++i)
        {
          m_qOut.data[i] = rgQ[i-31];
        }
        for(unsigned int i = 37; i < 42; ++i)
        {
          m_qOut.data[i] = lgQ[i-36];
        }
        /* FIXME Correction RPY convention here? */
        Eigen::Vector3d rpy = Eigen::Quaterniond(res.q[0], res.q[1], res.q[2], res.q[3]).toRotationMatrix().eulerAngles(2, 1, 0);
        m_rpyOut.data.r = rpy[2];
        m_rpyOut.data.p = rpy[1];
        m_rpyOut.data.y = rpy[0];

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
    unsigned int loop_t = (coiltmout.sec() - coiltm.sec())*1e6 + coiltmout.usec() - coiltm.usec();
    if(loop_t > max_t)
    {
      max_t = loop_t;
    }
    std::cout << "\rTime spent in controller: " << loop_t << " us" << std::flush;
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

