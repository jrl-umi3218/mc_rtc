/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <Eigen/StdVector>

#include <memory>

namespace mc_rbdyn
{

struct JointSensor;
using JointSensorPtr = std::unique_ptr<JointSensor>;

/** This structure defines a joint sensor that provides
 * temperature and current information for a specfic joint. */
struct MC_RBDYN_DLLAPI JointSensor
{
  /** Default constructor, does not represent a valid joint sensor */
  inline JointSensor() : JointSensor("", "") {}

  /** Constructor
   *
   * @param name Name of the sensor
   *
   * @param jointName Name of the joint to which the sensor is attached
   *
   */
  JointSensor(const std::string & name, const std::string & jointName)
  {
    name_ = name;
    joint_ = jointName;
    type_ = "JointSensor";
  }

  JointSensor(const JointSensor & js) : JointSensor(js.name(), js.parentJoint())
  {
    motor_temperature_ = js.motor_temperature_;
    driver_temperature_ = js.driver_temperature_;
    motor_current_ = js.motor_current_;
  }

  JointSensor & operator=(const JointSensor & js)
  {
    name_ = js.name_;
    joint_ = js.joint_;
    motor_temperature_ = js.motor_temperature_;
    driver_temperature_ = js.driver_temperature_;
    motor_current_ = js.motor_current_;
    return *this;
  }

  JointSensor(JointSensor &&) = default;
  JointSensor & operator=(JointSensor &&) = default;

  ~JointSensor() noexcept = default;

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

  /** Returns the sensor's joint name */
  inline const std::string & parentJoint() const
  {
    return joint_;
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

  /** Perform a device copy */
  JointSensorPtr clone() const;

protected:
  /** Name of the sensor */
  std::string name_;
  /** Name of joint to which sensor is attached */
  std::string joint_;
  /** Type of sensor as string */
  std::string type_;

private:
  double motor_temperature_ = 0;
  double driver_temperature_ = 0;
  double motor_current_ = 0;
};

typedef std::vector<JointSensor, Eigen::aligned_allocator<JointSensor>> JointSensorVector;

} // namespace mc_rbdyn
