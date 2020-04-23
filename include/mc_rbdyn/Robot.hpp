/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/type_name.h>

namespace mc_rbdyn
{

template<typename T>
const T * Robot::hasSensor(const std::string & name) const
{
  auto it = sensorsIndex_.find(name);
  if(it == sensorsIndex_.end())
  {
    return nullptr;
  }
  return dynamic_cast<T *>(sensors_[it->second]);
}

template<>
inline const ForceSensor * Robot::hasSensor<ForceSensor>(const std::string & name) const
{
  if(hasForceSensor(name))
  {
    return &(forceSensor(name));
  }
  return nullptr;
}

template<>
inline const BodySensor * Robot::hasSensor<BodySensor>(const std::string & name) const
{
  if(hasBodySensor(name))
  {
    return &(bodySensor(name));
  }
  return nullptr;
}

template<typename T>
const T & Robot::sensor(const std::string & name) const
{
  const T * s = this->hasSensor<T>(name);
  if(!s)
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "No sensor named "
                            << name << " in " << this->name()
                            << " or the sensor type did not match the requested one: " << mc_rtc::type_name<T>())
  }
  return *s;
}

} // namespace mc_rbdyn
