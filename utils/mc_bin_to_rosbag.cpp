/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/log/serialization/MCLog_generated.h>
#include <mc_rtc/logging.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>

#include <fstream>
#include <rosbag/bag.h>
#include <string>
#include <vector>

using namespace mc_rtc::log;

/** Convert flatbuffer union type to message type.
 *
 * The default template is declared but not defined.
 */
template<mc_rtc::log::LogData T>
struct ROSMessageFromFBUnion;

#define MAKE_RM_F_FB(FB_T, RM_T)     \
  template<>                         \
  struct ROSMessageFromFBUnion<FB_T> \
  {                                  \
    typedef RM_T message_t;          \
  }
MAKE_RM_F_FB(LogData_Bool, std_msgs::Bool);
MAKE_RM_F_FB(LogData_Double, std_msgs::Float64);
MAKE_RM_F_FB(LogData_DoubleVector, std_msgs::Float64MultiArray);
MAKE_RM_F_FB(LogData_UnsignedInt, std_msgs::UInt64);
MAKE_RM_F_FB(LogData_String, std_msgs::String);
MAKE_RM_F_FB(LogData_Vector2d, geometry_msgs::Vector3);
MAKE_RM_F_FB(LogData_Vector3d, geometry_msgs::Vector3);
MAKE_RM_F_FB(LogData_Quaterniond, geometry_msgs::Quaternion);
MAKE_RM_F_FB(LogData_PTransformd, geometry_msgs::Transform);
MAKE_RM_F_FB(LogData_ForceVecd, geometry_msgs::Wrench);
MAKE_RM_F_FB(LogData_MotionVecd, geometry_msgs::Twist);
#undef MAKE_RM_F_FB

template<mc_rtc::log::LogData T>
struct FBToROS
{
  using ret_t = typename ROSMessageFromFBUnion<T>::message_t;
  static ret_t convert(const void *);
};

template<>
FBToROS<LogData_Bool>::ret_t FBToROS<LogData_Bool>::convert(const void * data)
{
  auto fb_data = static_cast<const Bool *>(data);
  ret_t ret;
  ret.data = fb_data->b();
  return ret;
}

template<>
FBToROS<LogData_Double>::ret_t FBToROS<LogData_Double>::convert(const void * data)
{
  auto fb_data = static_cast<const Double *>(data);
  ret_t ret;
  ret.data = fb_data->d();
  return ret;
}

template<>
FBToROS<LogData_DoubleVector>::ret_t FBToROS<LogData_DoubleVector>::convert(const void * data)
{
  auto fb_data = static_cast<const DoubleVector *>(data);
  size_t fb_size = fb_data->v()->size();
  ret_t ret;
  ret.layout.dim.resize(1);
  ret.layout.dim[0].label = "data";
  ret.layout.dim[0].size = fb_size;
  ret.layout.dim[0].stride = fb_size;
  if(fb_size)
  {
    const auto & fb_v = *fb_data->v();
    for(size_t i = 0; i < fb_size; ++i)
    {
      ret.data.push_back(fb_v[i]);
    }
  }
  return ret;
}

template<>
FBToROS<LogData_UnsignedInt>::ret_t FBToROS<LogData_UnsignedInt>::convert(const void * data)
{
  auto fb_data = static_cast<const UnsignedInt *>(data);
  ret_t ret;
  ret.data = fb_data->i();
  return ret;
}

template<>
FBToROS<LogData_String>::ret_t FBToROS<LogData_String>::convert(const void * data)
{
  auto fb_data = static_cast<const String *>(data);
  ret_t ret;
  ret.data = fb_data->s()->str();
  return ret;
}

template<>
FBToROS<LogData_Vector2d>::ret_t FBToROS<LogData_Vector2d>::convert(const void * data)
{
  auto fb_data = static_cast<const Vector2d *>(data);
  ret_t ret;
  ret.x = fb_data->x();
  ret.y = fb_data->y();
  ret.z = 0.0;
  return ret;
}

template<>
FBToROS<LogData_Vector3d>::ret_t FBToROS<LogData_Vector3d>::convert(const void * data)
{
  auto fb_data = static_cast<const Vector3d *>(data);
  ret_t ret;
  ret.x = fb_data->x();
  ret.y = fb_data->y();
  ret.z = fb_data->z();
  return ret;
}

template<>
FBToROS<LogData_Quaterniond>::ret_t FBToROS<LogData_Quaterniond>::convert(const void * data)
{
  auto fb_data = static_cast<const Quaterniond *>(data);
  ret_t ret;
  ret.w = fb_data->w();
  ret.x = fb_data->x();
  ret.y = fb_data->y();
  ret.z = fb_data->z();
  return ret;
}

template<>
FBToROS<LogData_PTransformd>::ret_t FBToROS<LogData_PTransformd>::convert(const void * data)
{
  auto fb_data = static_cast<const PTransformd *>(data);
  ret_t ret;
  ret.rotation.w = fb_data->ori()->w();
  ret.rotation.x = fb_data->ori()->x();
  ret.rotation.y = fb_data->ori()->y();
  ret.rotation.z = fb_data->ori()->z();
  ret.translation.x = fb_data->pos()->x();
  ret.translation.y = fb_data->pos()->y();
  ret.translation.z = fb_data->pos()->z();
  return ret;
}

template<>
FBToROS<LogData_ForceVecd>::ret_t FBToROS<LogData_ForceVecd>::convert(const void * data)
{
  auto fb_data = static_cast<const ForceVecd *>(data);
  ret_t ret;
  ret.force.x = fb_data->force()->x();
  ret.force.y = fb_data->force()->y();
  ret.force.z = fb_data->force()->z();
  ret.torque.x = fb_data->couple()->x();
  ret.torque.y = fb_data->couple()->y();
  ret.torque.z = fb_data->couple()->z();
  return ret;
}

template<>
FBToROS<LogData_MotionVecd>::ret_t FBToROS<LogData_MotionVecd>::convert(const void * data)
{
  auto fb_data = static_cast<const MotionVecd *>(data);
  ret_t ret;
  ret.linear.x = fb_data->linear()->x();
  ret.linear.y = fb_data->linear()->y();
  ret.linear.z = fb_data->linear()->z();
  ret.angular.x = fb_data->angular()->x();
  ret.angular.y = fb_data->angular()->y();
  ret.angular.z = fb_data->angular()->z();
  return ret;
}

void mc_bin_to_rosbag(const std::string & in, const std::string & out, double dt)
{
  ros::Time::init();
  auto now = ros::Time::now();
  rosbag::Bag bag(out, rosbag::bagmode::Write);
  std::vector<std::string> topics;
  std::ifstream ifs(in, std::ifstream::binary);
  while(ifs)
  {
    int size = 0;
    ifs.read((char *)(&size), sizeof(int));
    if(ifs)
    {
      char * data = new char[size];
      ifs.read(data, size);
      auto log = mc_rtc::log::GetLog(data);
      if(log->keys())
      {
        topics.clear();
        const auto & nkeys = *log->keys();
        for(auto k : nkeys)
        {
          topics.push_back(k->str());
        }
      }
      const auto & values_type = *log->values_type();
      const auto & values = *log->values();
      for(size_t i = 0; i < topics.size(); ++i)
      {
        const auto & topic = topics[i];
        LogData type = LogData(values_type[i]);
        const void * fb_data = values[i];
        switch(type)
        {
#define CASE_ENUM(T)                                     \
  case T:                                                \
    bag.write(topic, now, FBToROS<T>::convert(fb_data)); \
    break
          CASE_ENUM(LogData_Bool);
          CASE_ENUM(LogData_Double);
          CASE_ENUM(LogData_DoubleVector);
          CASE_ENUM(LogData_UnsignedInt);
          CASE_ENUM(LogData_String);
          CASE_ENUM(LogData_Vector2d);
          CASE_ENUM(LogData_Vector3d);
          CASE_ENUM(LogData_Quaterniond);
          CASE_ENUM(LogData_PTransformd);
          CASE_ENUM(LogData_ForceVecd);
          CASE_ENUM(LogData_MotionVecd);
          default:
            break;
#undef CASE_ENUM
        };
      }
      delete[] data;
    }
    now += ros::Duration(dt);
  }
  bag.close();
}
