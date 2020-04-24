/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/type_name.h>

namespace mc_rbdyn
{

template<typename T>
bool Robot::hasSensor(const std::string & name) const
{
  return sensorsIndex_.count(name) != 0;
}

template<>
inline bool Robot::hasSensor<ForceSensor>(const std::string & name) const
{
  return hasForceSensor(name);
}

template<>
inline bool Robot::hasSensor<BodySensor>(const std::string & name) const
{
  return hasBodySensor(name);
}

template<typename T>
const T & Robot::sensor(const std::string & name) const
{
  auto it = sensorsIndex_.find(name);
  if(it == sensorsIndex_.end())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No sensor named " << name << " in " << this->name());
  }
  auto ptr = dynamic_cast<T *>(sensors_[it->second].get());
  if(!ptr)
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        name << " sensor type did not match the requested one: " << mc_rtc::type_name<T>())
  }
  return *ptr;
}

template<>
inline const ForceSensor & Robot::sensor<ForceSensor>(const std::string & name) const
{
  return forceSensor(name);
}

template<>
inline const BodySensor & Robot::sensor<BodySensor>(const std::string & name) const
{
  return bodySensor(name);
}

} // namespace mc_rbdyn
