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

//
#include <mc_rbdyn/RobotLoader.h>
#include <algorithm>

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

MCControlTCP::MCControlTCP(const std::string & host, mc_control::MCGlobalController & controller, const std::string & conf_joints_file):
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

  Json::Value v;
  std::ifstream ifs(conf_joints_file);
  if(ifs.bad())
  {
    LOG_ERROR("Failed to open joints configuration file: " << conf_joints_file)
  }
  try
  {
    ifs >> v;
  }
  catch(const std::runtime_error & exc)
  {
    LOG_ERROR("Failed to read configuration file")
    LOG_WARNING(exc.what())
  }
  if(v.isMember("Deactivated"))
  {
    std::string robot_name = m_controller.robot().name();
    std::transform(robot_name.begin(), robot_name.end(), robot_name.begin(), ::toupper);
    auto currentRobot =  mc_rbdyn::RobotLoader::get_robot_module(robot_name);
    const auto & halfSit = currentRobot->stance();
    for(const auto & cv: v["Deactivated"])
    {
      if(halfSit.find(cv.asString()) != halfSit.end())
      {
        deactivatedJoints.insert(
          std::pair<std::string, double>(cv.asString(), halfSit.at(cv.asString())[0]));
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

      //FIXME The controller's robot name should be the same as the module's name
      for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
      {
        if(deactivatedJoints.find(ref_joint_order[i]) != deactivatedJoints.end())
        {
          control_data.control[i] = deactivatedJoints.at(ref_joint_order[i]);
        }
        else
        {
          control_data.control[i] = res.robots_state[0].q.at(ref_joint_order[i])[0];
        }
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
    const auto & ref_joint_order = m_controller.ref_joint_order();
    for(unsigned i = 0; i < sensors_traits<Tsensor>::dof; ++i)
    {
      if(deactivatedJoints.find(ref_joint_order[i]) != deactivatedJoints.end())
      {
        qIn[i] = deactivatedJoints.at(ref_joint_order[i]);
      }
      else
      {
        qIn[i] = data.position[i];
      }
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
    m_wrenches[m_wrenchesNames[0]].force()=Eigen::Vector3d(data.forceRF[0],data.forceRF[1],data.forceRF[2]);
    m_wrenches[m_wrenchesNames[0]].couple()=Eigen::Vector3d(data.forceRF[3],data.forceRF[4],data.forceRF[5]);
    m_wrenches[m_wrenchesNames[1]].force()=Eigen::Vector3d(data.forceLF[0],data.forceLF[1],data.forceLF[2]);
    m_wrenches[m_wrenchesNames[1]].couple()=Eigen::Vector3d(data.forceLF[3],data.forceLF[4],data.forceLF[5]);
    m_wrenches[m_wrenchesNames[2]].force()=Eigen::Vector3d(data.forceRH[0],data.forceRH[1],data.forceRH[2]);
    m_wrenches[m_wrenchesNames[2]].couple()=Eigen::Vector3d(data.forceRH[3],data.forceRH[4],data.forceRH[5]);
    m_wrenches[m_wrenchesNames[3]].force()=Eigen::Vector3d(data.forceLH[0],data.forceLH[1],data.forceLH[2]);
    m_wrenches[m_wrenchesNames[3]].couple()=Eigen::Vector3d(data.forceLH[3],data.forceLH[4],data.forceLH[5]);

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

    m_controller.setSensorOrientation(rpyIn);
    m_controller.setSensorAcceleration(accIn);
    m_controller.setSensorAngularVelocity(rateIn);
    m_controller.setEncoderValues(qIn);
    m_controller.setWrenches(m_wrenches);
    if(!init)
    {
      LOG_INFO("Init controller");
      m_controller.init(qIn);
      init = true;
    }
  }
}

mc_control::MCGlobalController & MCControlTCP::controller()
{
  return m_controller;
}
