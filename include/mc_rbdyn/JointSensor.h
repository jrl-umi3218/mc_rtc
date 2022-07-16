/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Device.h>
#include <mc/rtc/deprecated.hh>

#include <Eigen/StdVector>

namespace mc_rbdyn
{

/** This structure defines a joint sensor that provides
 * temperature and current information for a specfic joint. */
struct MC_RBDYN_DLLAPI JointSensor : public Device
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid joint sensor */
  inline JointSensor() : JointSensor("", "") {}

  /** Constructor
   *
   * @param name Name of the sensor
   *
   * @param jointName Name of the joint to which the sensor is attached
   *
   */
  inline JointSensor(const std::string & name, const std::string & jointName) : Device(name, jointName)
  {
    type_ = "JointSensor";
  }

  JointSensor(const JointSensor & js) : JointSensor(js.name(), js.parent())
  {
    motor_temperature_ = js.motor_temperature_;
    driver_temperature_ = js.driver_temperature_;
    motor_current_ = js.motor_current_;
  }

  JointSensor & operator=(const JointSensor & js)
  {
    name_ = js.name_;
    parent_ = js.parent_;
    motor_temperature_ = js.motor_temperature_;
    driver_temperature_ = js.driver_temperature_;
    motor_current_ = js.motor_current_;
    return *this;
  }

  JointSensor(JointSensor &&) = default;
  JointSensor & operator=(JointSensor &&) = default;

  ~JointSensor() noexcept override;

  /** Get the sensor's parent joint name */
  inline const std::string & parentJoint() const
  {
    return Device::parent();
  }

  /** Return the sensor's motor temperature reading, Zero if not provided */
  inline const double & motorTemperature() const
  {
    return motor_temperature_;
  }

  /** Set the sensor's motor temperature reading */
  inline void motorTemperature(const double & motor_temperature)
  {
    motor_temperature_ = motor_temperature;
  }

  /** Return the sensor's driver temperature reading, Zero if not provided */
  inline const double & driverTemperature() const
  {
    return driver_temperature_;
  }

  /** Set the sensor's driver temperature reading */
  inline void driverTemperature(const double & driver_temperature)
  {
    driver_temperature_ = driver_temperature;
  }

  /** Return the sensor's current reading, Zero if not provided */
  inline const double & motorCurrent() const
  {
    return motor_current_;
  }

  /** Set the sensor's current reading */
  inline void motorCurrent(const double & motor_current)
  {
    motor_current_ = motor_current;
  }

  DevicePtr clone() const override;

private:
  double motor_temperature_ = 0;
  double driver_temperature_ = 0;
  double motor_current_ = 0;
};

typedef std::vector<JointSensor, Eigen::aligned_allocator<JointSensor>> JointSensorVector;

} // namespace mc_rbdyn
