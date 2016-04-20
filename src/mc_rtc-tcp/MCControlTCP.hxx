#pragma once

#include "MCControlTCP.h"

#include <mc_rtc/ros.h>

// std
#include <chrono>
#include <thread>

// boost
#include <boost/asio.hpp>
#include <boost/thread.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

template<class Tsensor>
SensorClient<Tsensor>::SensorClient(const std::string & host, const std::string & port, typename ReadAndAck<Tsensor>::callback_t callback) :
  data_init_(createSensors<Tsensor>()),
  io_service_(),
  proto_(io_service_, data_init_, callback),
  client_(io_service_, host, port, proto_),
  thread_(boost::bind(&SensorClient<Tsensor>::thread, this))
{
}

template<class Tsensor>
SensorClient<Tsensor>::~SensorClient()
{
  io_service_.stop();
  thread_.join();
}

template<class Tsensor>
void SensorClient<Tsensor>::thread()
{
  boost::asio::io_service::work work(io_service_);
  io_service_.run();
}

template<class Tcontrol>
ControlClient<Tcontrol>::ControlClient(const std::string & host, const std::string & port, const Tcontrol & init) :
  io_service_(),
  proto_(io_service_, init),
  client_(io_service_, host, port, proto_),
  thread_(boost::bind(&ControlClient<Tcontrol>::thread, this))
{
}

template<class Tcontrol>
ControlClient<Tcontrol>::~ControlClient()
{
  io_service_.stop();
  thread_.join();
}

template<class Tcontrol>
WriteAndAck<Tcontrol> & ControlClient<Tcontrol>::proto()
{
  return proto_;
}

template<class Tsensor>
void ControlClient<Tsensor>::thread()
{
  boost::asio::io_service::work work(io_service_);
  io_service_.run();
}

MCControlTCP::MCControlTCP(const std::string & host, mc_control::MCGlobalController & controller):
    m_controller(controller),
    m_service(this->m_controller),
    m_timeStep(ceil(1000*controller.timestep())),
    m_running(true), init(false),
    m_wrenchesNames(controller.robot().forceSensorsByName()),
    iter_since_start(0),
    host(host),
    strPortSensor(""),
    strPortControl("")
{
  auto gripperJs = controller.gripperJoints();
  auto gripperActiveJs = controller.gripperActiveJoints();
  const auto & ref_joint_order = controller.ref_joint_order();
  for(const auto & g : gripperActiveJs)
  {
    gripper_in_index[g.first] = {};
    realGripperQs[g.first] = {};
    for(const auto & jn : g.second)
    {
      for(size_t i = 0; i < ref_joint_order.size(); ++i)
      {
        if(ref_joint_order[i] == jn)
        {
          gripper_in_index[g.first].push_back(i);
          realGripperQs[g.first].push_back(0.0);
        }
      }
    }
  }
  for(const auto & g : gripperJs)
  {
    gripper_out_index[g.first] = {};
    for(size_t j = 0; j < g.second.size(); ++j)
    {
      const auto & jn = g.second[j];
      for(size_t i = 0; i < ref_joint_order.size(); ++i)
      {
        if(ref_joint_order[i] == jn)
        {
          gripper_out_index[g.first].push_back({i, j});
        }
      }
    }
  }
}

int MCControlTCP::initialize()
{
  strPortSensor = boost::lexical_cast<std::string>("4002");
  strPortControl = boost::lexical_cast<std::string>("4003");
  return 0;
}


template <class Tsensor, class Tcontrol>
void MCControlTCP::start()
{
  Tcontrol control_data = createControl<Tcontrol>();
  LOG_INFO("Creating the client for the sensor")
  SensorClient<Tsensor> sc(host, strPortSensor, boost::bind(&MCControlTCP::sensorCallback<Tsensor>, this, _1));

  LOG_INFO("Creating the client for the control");
  ControlClient<Tcontrol> cc(host, strPortControl, control_data);

  m_controller.running = true;

  while(m_running)
  {
    auto start = std::chrono::high_resolution_clock::now();
    controlCallback<Tcontrol>(cc.proto(), control_data);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count();
    std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - elapsed));
  }
}

bool MCControlTCP::running()
{
  return m_running;
}

void MCControlTCP::stop()
{
  m_running = false;
}

template<class Tcontrol>
void MCControlTCP::controlCallback(WriteAndAck<Tcontrol>& control_proto, Tcontrol & control_data)
{
  control_data.hasZmp = false;
  control_data.hasBaseFF = false;

  if(m_controller.running && init)
  {
    if(m_controller.run())
    {
      //FIXME Fill t
      double t = 0.; //in nano second
      const mc_control::QPResultMsg & res = m_controller.send(t);
      const auto & ref_joint_order = m_controller.ref_joint_order();
      //FIXME The index correspondance should be computed only one time
      auto gripperQs = m_controller.gripperQ();
      for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
      {
        control_data.control[i] = res.robots_state[0].q.at(ref_joint_order[i])[0];
      }
      /* Update gripper state */
      for(const auto & cG : gripper_out_index)
      {
        const auto & qs = gripperQs[cG.first];
        for(const auto & idx_p : cG.second)
        {
          control_data.control[idx_p.first] = qs[idx_p.second];
        }
      }
    }
    mc_rtc::ROSBridge::update_robot_publisher(m_controller.timestep(), m_controller.robot(), Eigen::Vector3d::Zero(), rpyIn, rateIn, accIn, m_controller.gripperJoints(), m_controller.gripperQ());
    log_data(control_data.control);
    iter_since_start++;
  }
  control_proto.data(control_data);
  control_proto.send();
}


template<class Tsensor>
void MCControlTCP::sensorCallback(const Tsensor& data)
{
  // Joint states and gripper states
  if(m_controller.running)
  {
    qIn.resize(m_controller.robot().mb().nrDof());
    for(unsigned i = 0; i < sensors_traits<Tsensor>::dof; ++i)
    {
      qIn[i] = data.position[i];
    }
    auto gripperQs = m_controller.gripperQ();
    for(auto & rG : realGripperQs)
    {
      const auto & idx = gripper_in_index[rG.first];
      auto & qs = rG.second;
      for(size_t i = 0; i < idx.size(); ++i)
      {
        qs[i] = data.position[idx[i]];
      }
    }
    m_controller.setActualGripperQ(realGripperQs);

    // Wrench
    m_wrenches[m_wrenchesNames[0]].force()=Eigen::Vector3d(data.forceLF[0],data.forceLF[1],data.forceLF[2]);
    m_wrenches[m_wrenchesNames[0]].couple()=Eigen::Vector3d(data.forceLF[3],data.forceLF[4],data.forceLF[5]);
    m_wrenches[m_wrenchesNames[1]].force()=Eigen::Vector3d(data.forceRF[0],data.forceRF[1],data.forceRF[2]);
    m_wrenches[m_wrenchesNames[1]].couple()=Eigen::Vector3d(data.forceRF[3],data.forceRF[4],data.forceRF[5]);
    m_wrenches[m_wrenchesNames[2]].force()=Eigen::Vector3d(data.forceLH[0],data.forceLH[1],data.forceLH[2]);
    m_wrenches[m_wrenchesNames[2]].couple()=Eigen::Vector3d(data.forceLH[3],data.forceLH[4],data.forceLH[5]);
    m_wrenches[m_wrenchesNames[3]].force()=Eigen::Vector3d(data.forceRH[0],data.forceRH[1],data.forceRH[2]);
    m_wrenches[m_wrenchesNames[3]].couple()=Eigen::Vector3d(data.forceRH[3],data.forceRH[4],data.forceRH[5]);

    // rpy
    rpyIn(0) = data.rpy[0];
    rpyIn(1) = data.rpy[1];
    rpyIn(2) = data.rpy[2];

    // acceleration
    accIn(0) = data.accelerometer[0];
    accIn(1) = data.accelerometer[1];
    accIn(2) = data.accelerometer[2];

    // angular velocity
    rateIn(0) = data.gyrometer[0];
    rateIn(1) = data.gyrometer[1];
    rateIn(2) = data.gyrometer[2];

    if(!init)
    {
      LOG_INFO("Init controller");
      m_controller.init(qIn);
      init = true;
      log_header();
    }

    m_controller.setSensorOrientation(rpyIn);
    m_controller.setSensorAcceleration(accIn);
    m_controller.setEncoderValues(qIn);
    m_controller.setWrenches(m_wrenches);
  }
}

void MCControlTCP::log_header()
{
  m_log.open("/tmp/mc-control.log");
  m_log << "t";
  for(unsigned int i = 0; i < qIn.size(); ++i)
  {
    m_log << ";qIn" << i;
  }
  for(unsigned int i = 0; i < qIn.size(); ++i)
  {
    m_log << ";qOut" << i;
  }
  for(const auto & wn : m_wrenchesNames)
  {
    m_log << ";" << wn << "_fx";
    m_log << ";" << wn << "_fy";
    m_log << ";" << wn << "_fz";
    m_log << ";" << wn << "_cx";
    m_log << ";" << wn << "_cy";
    m_log << ";" << wn << "_cz";
  }
  m_controller.log_header(m_log);
  m_log << std::endl;
}

void MCControlTCP::log_data(const double * qOut)
{
  if(m_log.is_open())
  {
    m_log << iter_since_start;
    for(const auto & qi : qIn)
    {
      m_log << ";" << qi;
    }
    for(size_t i = 0; i < qIn.size(); ++i)
    {
      m_log << ";" << qOut[i];
    }
    for(const auto & w : m_wrenches)
    {
      m_log << ";" << w.second.force().x();
      m_log << ";" << w.second.force().y();
      m_log << ";" << w.second.force().z();
      m_log << ";" << w.second.couple().x();
      m_log << ";" << w.second.couple().y();
      m_log << ";" << w.second.couple().z();
    }
    m_controller.log_data(m_log);
    m_log << std::endl;
  }
}
