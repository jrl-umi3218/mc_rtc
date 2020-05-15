/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <memory>
#include <string>

namespace mc_rbdyn
{

struct Robot;

struct Device;
using DevicePtr = std::unique_ptr<Device>;
using Sensor = Device;
using SensorPtr = DevicePtr;

/** This struct represents a generic device attached to a robot
 *
 * This is a barebone interface meant to be derived by a concrete device implementation
 *
 */
struct MC_RBDYN_DLLAPI Device
{
  Device(const std::string & name, const std::string & parent, const sva::PTransformd & X_p_s);

  virtual ~Device() noexcept = default;

  /** Returns the name of the sensor */
  inline const std::string & name() const
  {
    return name_;
  }

  /** Returns the type of the sensor */
  inline const std::string & type() const
  {
    return type_;
  }

  /** Returns the parent body of the sensor */
  inline const std::string & parent() const
  {
    return parent_;
  }

  /** Returns the transformation from the parent body to the sensor */
  inline const sva::PTransformd & X_p_s() const
  {
    return X_p_s_;
  }

  /** Returns the sensor position in the inertial frame (convenience function) */
  sva::PTransformd X_0_s(const mc_rbdyn::Robot & robot) const;

  virtual DevicePtr clone() const = 0;

protected:
  /** Type of sensor as string */
  std::string type_;
  /** Name of the sensor */
  std::string name_;
  /** Parent body of the sensor */
  std::string parent_;
  /** Transformation from the parent body frame to the sensor frame */
  sva::PTransformd X_p_s_;
};

} // namespace mc_rbdyn
