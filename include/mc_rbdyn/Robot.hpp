/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/type_name.h>

namespace mc_rbdyn
{

template<typename T>
bool Robot::hasDevice(const std::string & name) const
{
  return devicesIndex_.count(name) != 0;
}

template<>
inline bool Robot::hasDevice<ForceSensor>(const std::string & name) const
{
  return hasForceSensor(name);
}

template<>
inline bool Robot::hasDevice<BodySensor>(const std::string & name) const
{
  return hasBodySensor(name);
}

template<typename T>
const T & Robot::device(const std::string & name) const
{
  auto it = devicesIndex_.find(name);
  if(it == devicesIndex_.end())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("No sensor named {} in {}", name, this->name());
  }
  auto ptr = dynamic_cast<T *>(devices_[it->second].get());
  if(!ptr)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("{} sensor type did not match the requested one: {}", name,
                                                     mc_rtc::type_name<T>());
  }
  return *ptr;
}

template<>
inline const ForceSensor & Robot::device<ForceSensor>(const std::string & name) const
{
  return forceSensor(name);
}

template<>
inline const BodySensor & Robot::device<BodySensor>(const std::string & name) const
{
  return bodySensor(name);
}

} // namespace mc_rbdyn
