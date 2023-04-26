/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/type_name.h>

namespace mc_rbdyn
{

template<typename T>
bool Robot::hasDevice(const std::string & name) const noexcept
{
  return data_->devicesIndex.count(name) != 0;
}

template<>
inline bool Robot::hasDevice<ForceSensor>(const std::string & name) const noexcept
{
  return hasForceSensor(name);
}

template<>
inline bool Robot::hasDevice<BodySensor>(const std::string & name) const noexcept
{
  return hasBodySensor(name);
}

template<typename T>
const T & Robot::device(const std::string & name) const
{
  auto it = data_->devicesIndex.find(name);
  if(it == data_->devicesIndex.end()) { mc_rtc::log::error_and_throw("No device named {} in {}", name, this->name()); }
  auto dev = data_->devices[it->second].get();
  auto ptr = dynamic_cast<T *>(dev);
  if(!ptr)
  {
    mc_rtc::log::error_and_throw("{} sensor type did not match the requested one: {} (actual device type: {})", name,
                                 mc_rtc::type_name<T>(), dev->type());
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
