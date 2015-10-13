// -*- C++ -*-
/*!
 * @file  MCControl.h * @brief Core component for MC control * @date  $Date$ 
 *
 * $Id$ 
 */
#ifndef MCCONTROL_H
#define MCCONTROL_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "MCControlServiceSVC_impl.h"

#include <mc_control/mc_global_controller.h>
#include <mc_rtc/ros.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

class MCControl  : public RTC::DataFlowComponentBase
{
 public:
  MCControl(RTC::Manager* manager);
  ~MCControl();

  // The initialize action (on CREATED->ALIVE transition)
  // formaer rtc_init_entry() 
 virtual RTC::ReturnCode_t onInitialize();

  // The finalize action (on ALIVE->END transition)
  // formaer rtc_exiting_entry()
  // virtual RTC::ReturnCode_t onFinalize();

  // The startup action when ExecutionContext startup
  // former rtc_starting_entry()
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  // The shutdown action when ExecutionContext stop
  // former rtc_stopping_entry()
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  // The activated action (Active state entry action)
  // former rtc_active_entry()
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  // The deactivated action (Active state exit action)
  // former rtc_active_exit()
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  // The execution action that is invoked periodically
  // former rtc_active_do()
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  // The aborting action when main logic error occurred.
  // former rtc_aborting_entry()
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  // The error action in ERROR state
  // former rtc_error_do()
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  // The reset action that is invoked resetting
  // This is same but different the former rtc_init_entry()
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  // The state update action that is invoked after onExecute() action
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  // The action that is invoked when execution context's rate is changed
  // no corresponding operation exists in OpenRTm-aist-0.2.0
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // Configuration variable declaration
  // <rtc-template block="config_declare">
  double m_timeStep;
  bool m_enabled;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  TimedDoubleSeq m_qIn;
  InPort<TimedDoubleSeq> m_qInIn;
  std::vector<double> qIn;
  TimedPoint3D m_pIn;
  InPort<TimedPoint3D> m_pInIn;
  TimedOrientation3D m_rpyIn;
  InPort<TimedOrientation3D> m_rpyInIn;
  TimedPose3D m_poseIn;
  InPort<TimedPose3D> m_poseInIn;
  Eigen::Vector3d rpyIn;

  std::vector<std::string> m_wrenchesNames;
  std::vector<TimedDoubleSeq*> m_wrenchesIn;
  std::vector< InPort<TimedDoubleSeq>* > m_wrenchesInIn;
  std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > m_wrenches;

  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  TimedDoubleSeq m_qOut;
  OutPort<TimedDoubleSeq> m_qOutOut;
  TimedPoint3D m_pOut;
  OutPort<TimedPoint3D> m_pOutOut;
  TimedOrientation3D m_rpyOut;
  OutPort<TimedOrientation3D> m_rpyOutOut;

  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_MCControlServicePortPort;

  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  OpenHRP::MCControlServiceSVC_impl m_service0;

  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">

  // </rtc-template>
 private:
public:
  mc_control::MCGlobalController controller;
  std::shared_ptr<mc_rtc::RobotPublisher> rpub;
  bool init;
};


extern "C"
{
  DLL_EXPORT void MCControlInit(RTC::Manager* manager);
};

#endif // MCCONTROL_H

