#ifndef MCCONTROLTCP_H
#define MCCONTROLTCP_H

#include <mc_control/mc_global_controller.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

// std
#include <fstream>

// boost
#include <boost/thread.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// tcp-com
#include <tcp_com/client_server.h>

// tcp-control
#include <tcp_control/controlStructures.h>

#include <chrono>


mc_control::MCGlobalController controller;
bool init;

void cltThread(boost::asio::io_service& io_service)
{
  // add a work to avoid io_service to finish when it has nothing to do
  boost::asio::io_service::work work(io_service);
  io_service.run();
}

template<class Tsensor, class Tcontrol>
void controlCallback(WriteAndAck<Tcontrol>& control_proto, Tcontrol control_data);

template<class Tsensor, class Tcontrol>
void sensorCallback(const Tsensor& data);

template<class Tsensor, class Tcontrol>
class SensorClient
{
public:
  SensorClient(const std::string & host, const std::string & strPortSensor)
  : sensor_data_init_(createSensors<Tsensor>())
  , sensor_proto_(io_sensor_srv_, sensor_data_init_,
                  boost::bind(sensorCallback<Tsensor, Tcontrol>, _1))
  , sensor_client_(io_sensor_srv_, host, strPortSensor, sensor_proto_)
  , sensor_thread_(cltThread, boost::ref(io_sensor_srv_))
  {}

  ~SensorClient()
  {
    io_sensor_srv_.stop();
    sensor_thread_.join();
  }

public:
  // sensor thread
  Tsensor sensor_data_init_;
  boost::asio::io_service io_sensor_srv_;
  ReadAndAck<Tsensor> sensor_proto_;
  Client<ReadAndAck<Tsensor> > sensor_client_;
  boost::thread sensor_thread_;
};

template<class T>
class ControlClient
{
public:
  ControlClient(const std::string & host, const std::string & strPortControl,
                const T& init)
    : control_proto_(io_control_srv_, init)
    , control_client_(io_control_srv_, host, strPortControl, control_proto_)
    , control_thread_(cltThread, boost::ref(io_control_srv_))
  {

  }

  ~ControlClient()
  {
    io_control_srv_.stop();
    control_thread_.join();
  }

public:
  // control thread
  boost::asio::io_service io_control_srv_;
  WriteAndAck<T> control_proto_;
  Client<WriteAndAck<T> > control_client_;
  boost::thread control_thread_;
};


template <class Tsensor, class Tcontrol>
class MCControlTCP
{
 public:

  MCControlTCP():
      m_timeStep(0.005),
      m_enabled(false),
      m_wrenchesNames({"RightFootForceSensor", "LeftFootForceSensor", "RightHandForceSensor", "LeftHandForceSensor"}),
      host("hrp4005c"),
      strPortSensor(""),
      strPortControl(""),
      control_data(createControl<Tcontrol>()),
      stabilizer(false)
  {}

  ~MCControlTCP()
  {}


  int initialize(int argc, char **argv)
  {
    strPortSensor = boost::lexical_cast<std::string>("4002");
    strPortControl = boost::lexical_cast<std::string>("4003");
    return 0;
  }


  void start(const std::string& robotname)
  {
    LOG_INFO("Creating the client for the sensor")
    SensorClient<Tsensor, Tcontrol> sc(host, strPortSensor);

    while(!init);

    LOG_INFO("Creating the client for the control");
    ControlClient<Tcontrol> cc(host, strPortControl, control_data);

    while(true)
    {
      auto start = std::chrono::high_resolution_clock::now();
      // if(stabilizer_)
      // {
      //   stabControlCallback<Tcontrol>, _1, _2,
      //                             boost::ref(cc.control_proto_),
      //                             boost::ref(control_data),));
      // }
      // else
      // {
      controlCallback<Tsensor, Tcontrol>(cc.control_proto_, control_data);
      // }
      //auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      //                std::chrono::high_resolution_clock::now()-start).count();
      //std::this_thread::sleep_for(std::chrono::milliseconds(m_timeStep - elapsed));
    }
  }


 protected:
  double m_timeStep;
  bool m_enabled;

  std::vector<std::string> m_wrenchesNames;


private:
  std::ofstream m_log;
  void log_header();
  void log_data();
  int count;

  std::string host;
  std::string strPortSensor;
  std::string strPortControl;

  bool stabilizer;
  Tcontrol control_data;

public:
};


template<class Tsensor, class Tcontrol>
void controlCallback(WriteAndAck<Tcontrol>& control_proto, Tcontrol control_data)
{
  control_data.hasZmp = false;
  control_data.hasBaseFF = false;

  if(controller.running && init)
  {
    if(controller.run())
    {
      //FIXME Fill t
      double t = 0.; //in nano second
      const mc_control::QPResultMsg & res = controller.send(t);
      const auto & ref_joint_order = controller.ref_joint_order();
      //FIXME The index correspondance should be computed only one time
      auto gripperQs = controller.gripperQ();
      auto gripperJs = controller.gripperJoints();
      auto gripperActiveJs = controller.gripperActiveJoints();
      for(unsigned int i = 0; i < ref_joint_order.size(); ++i)
      {
        control_data.control[i] = res.robots_state[0].q.at(ref_joint_order[i])[0];
      }
      /* Update gripper state */
      for(const auto & g : gripperJs)
      {
        const auto & gName = g.first;
        const auto & jNames = g.second;
        const auto & gQ = gripperQs[gName];
        for(size_t i = 0; i < jNames.size(); ++i)
        {
          for(size_t j = 0; j < ref_joint_order.size(); ++j)
          {
            if(ref_joint_order[j] == jNames[i])
            {
              control_data.control[j] =  gQ[i];
              break;
            }
          }
        }
      }
    }
    //log_data();
  }
  control_proto.data(control_data);
  control_proto.send();
}


template<class Tsensor, class Tcontrol>
void sensorCallback(const Tsensor& data)
{
  // joint state
  if(controller.running)
  {
    std::vector<double> qIn(controller.robot().mb().nrDof());
    for(unsigned i = 0; i < sensors_traits<Tsensor>::dof; ++i)
      qIn[i] = data.position[i];

    const auto & ref_joint_order = controller.ref_joint_order();
    auto gripperActiveJs = controller.gripperActiveJoints();
    std::map<std::string, std::vector<double>> realGripperQs;
    for(const auto & g : gripperActiveJs)
    {
      realGripperQs[g.first] = {};
      for(const auto & jn : g.second)
      {
        for(size_t i = 0; i < ref_joint_order.size(); ++i)
        {
          if(ref_joint_order[i] == jn)
          {
            realGripperQs[g.first].push_back(data.position[i]);
            break;
          }
        }
      }
    }
    controller.setActualGripperQ(realGripperQs);


    // wrench
    std::vector<sva::ForceVecd> m_wrenches(4);
    m_wrenches[0].force()=Eigen::Vector3d(data.forceLH[0],data.forceLH[1],data.forceLH[2]);
    m_wrenches[0].couple()=Eigen::Vector3d(data.forceLH[3],data.forceLH[4],data.forceLH[5]);
    m_wrenches[1].force()=Eigen::Vector3d(data.forceRH[0],data.forceRH[1],data.forceRH[2]);
    m_wrenches[1].couple()=Eigen::Vector3d(data.forceRH[3],data.forceRH[4],data.forceRH[5]);
    m_wrenches[2].force()=Eigen::Vector3d(data.forceLF[0],data.forceLF[1],data.forceLF[2]);
    m_wrenches[2].couple()=Eigen::Vector3d(data.forceLF[3],data.forceLF[4],data.forceLF[5]);
    m_wrenches[3].force()=Eigen::Vector3d(data.forceRF[0],data.forceRF[1],data.forceRF[2]);
    m_wrenches[3].couple()=Eigen::Vector3d(data.forceRF[3],data.forceRF[4],data.forceRF[5]);

    // rpy
    Eigen::Vector3d rpyIn;
    rpyIn(0) = data.rpy[0];
    rpyIn(1) = data.rpy[1];
    rpyIn(2) = data.rpy[2];

    // acceleration
    Eigen::Vector3d accIn;
    accIn(0) = data.accelerometer[0];
    accIn(1) = data.accelerometer[1];
    accIn(2) = data.accelerometer[2];

    // angular velocity
    Eigen::Vector3d rateIn;
    rateIn(0) = data.gyrometer[0];
    rateIn(1) = data.gyrometer[1];
    rateIn(2) = data.gyrometer[2];

    std::vector<double> taucIn;

    if(!init)
    {
      LOG_INFO("Init controller");
      //FIXME
      //control_data.control = qIn;
      controller.init(qIn);
      init = true;
      //MCControlTCP<Tsensor, Tcontrol>::log_header();
    }

    controller.setSensorOrientation(rpyIn);
    controller.setSensorAcceleration(accIn);
    controller.setEncoderValues(qIn);
    controller.setWrenches(m_wrenches);

    //mc_rtc::ROSBridge::update_robot_publisher(controller.timestep(), controller.robot(), pIn, rpyIn, rateIn, accIn, controller.gripperJoints(), controller.gripperQ());
  }
}



#include "MCControlTCP.cpp"

#endif // MCCONTROLTCP_H


