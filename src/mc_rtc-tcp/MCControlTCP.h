#pragma once

// mc_rtc
#include <mc_control/mc_global_controller.h>

// boost
#include <boost/thread.hpp>

// tcp-com
#include <tcp_com/client_server.h>

// tcp-control
#include <tcp_control/controlStructures.h>

// std
#include <fstream>

// service provider
#include "MCControlTCPService.h"

template<class Tsensor>
class SensorClient
{
public:
  SensorClient(const std::string & host, const std::string & port, typename ReadAndAck<Tsensor>::callback_t callback);

  ~SensorClient();

  void thread();
private:
  Tsensor data_init_;
  boost::asio::io_service io_service_;
  ReadAndAck<Tsensor> proto_;
  Client<ReadAndAck<Tsensor> > client_;
  boost::thread thread_;
};

template<class Tcontrol>
class ControlClient
{
public:
  ControlClient(const std::string & host, const std::string & port, const Tcontrol & init);

  ~ControlClient();

  WriteAndAck<Tcontrol> & proto();

  void thread();
private:
  boost::asio::io_service io_service_;
  WriteAndAck<Tcontrol> proto_;
  Client<WriteAndAck<Tcontrol>> client_;
  boost::thread thread_;
};

class MCControlTCP
{
public:
  MCControlTCP(const std::string & host, mc_control::MCGlobalController & controller,
                    const std::string & conf_joints_file);

  int initialize();

  template<class Tsensor, class Tcontrol>
  void start();

  template<class Tsensor>
  void sensorCallback(const Tsensor & data);

  template<class Tcontrol>
  void controlCallback(WriteAndAck<Tcontrol> & proto, Tcontrol & data);

  bool running();

  void stop();

  mc_control::MCGlobalController & controller();

private:
  void log_header();
  void log_data(const double * qOut);
private:
  mc_control::MCGlobalController & m_controller;
  MCControlTCPService m_service;
  /*! Timestep expressed in ms */
  unsigned int m_timeStep;
  bool m_running;
  bool init;
  /* Sensor information */
  /*! Encoder values */
  std::vector<double> qIn;
  /*! Names of force sensors */
  std::vector<std::string> m_wrenchesNames;
  /*! Value of force sensors */
  std::map<std::string, sva::ForceVecd> m_wrenches;
  /*! Orientation sensor */
  Eigen::Vector3d rpyIn;
  /*! Accelerometer */
  Eigen::Vector3d accIn;
  /*! Angular velocity */
  Eigen::Vector3d rateIn;
  /*! Log file */
  std::ofstream m_log;
  /*! Controller's iteration count*/
  unsigned int iter_since_start;

  /* Connection information */
  /*! Connection host */
  std::string host;
  /*! Remote port for sensor connection */
  std::string strPortSensor;
  /*! Remote port for control connection */
  std::string strPortControl;

  /* For each gripper, store the joints to report as the actual gripper values */
  std::map<std::string, std::vector<size_t>> gripper_in_index;
  std::map<std::string, std::vector<double>> realGripperQs;
  std::map<std::string, std::vector<std::pair<size_t, size_t>>> gripper_out_index;

  /*! Deactivated Joints */
  std::map<std::string, double> deactivatedJoints;
};

#include "MCControlTCP.hxx"
