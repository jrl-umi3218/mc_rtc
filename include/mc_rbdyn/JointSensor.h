/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <limits>
#include <memory>
#include <string>

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

  /** Constructor
   *
   * @param jointName Name of the joint to which the sensor is attached
   *
   */
  JointSensor(const std::string & jointName);

  JointSensor(const JointSensor & js) : JointSensor(js.name(), js.joint())
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
  inline const std::string & joint() const
  {
    return joint_;
  }

  /** Return the sensor's motor temperature reading (Celcius), NaN if not provided */
  inline double motorTemperature() const
  {
    return motor_temperature_;
  }

  /** Set the sensor's motor temperature reading (Celcius) */
  inline void motorTemperature(double motor_temperature)
  {
    motor_temperature_ = motor_temperature;
  }

  /** Return the sensor's driver temperature reading (Celcius), NaN if not provided */
  inline double driverTemperature() const
  {
    return driver_temperature_;
  }

  /** Set the sensor's driver temperature reading (Celcius) */
  inline void driverTemperature(double driver_temperature)
  {
    driver_temperature_ = driver_temperature;
  }

  /** Return the sensor's current reading (Ampere), NaN if not provided */
  inline double motorCurrent() const
  {
    return motor_current_;
  }

  /** Set the sensor's current reading (Ampere) */
  inline void motorCurrent(double motor_current)
  {
    motor_current_ = motor_current;
  }

protected:
  /** Name of the sensor */
  std::string name_;
  /** Name of joint to which sensor is attached */
  std::string joint_;
  /** Type of sensor as string */
  std::string type_;

private:
  /* Motor temperature (Celcius) */
  double motor_temperature_ = std::numeric_limits<double>::quiet_NaN();
  /* Drive temperature (Celcius) */
  double driver_temperature_ = std::numeric_limits<double>::quiet_NaN();
  /* Motor current (Ampere) */
  double motor_current_ = std::numeric_limits<double>::quiet_NaN();
};

} // namespace mc_rbdyn
