/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>

#include <limits>
#include <string>

namespace mc_rbdyn
{

/** This structure defines a joint sensor that provides
 * temperature and current information for a specfic joint. */
struct MC_RBDYN_DLLAPI JointSensor
{
  /** Default constructor, does not represent a valid joint sensor */
  inline JointSensor() : JointSensor("") {}

  /** Constructor
   *
   * @param jointName Name of the joint to which the sensor is attached
   *
   */
  inline JointSensor(const std::string & jointName) { joint_ = jointName; }

  JointSensor(const JointSensor &) = default;
  JointSensor & operator=(const JointSensor &) = default;

  JointSensor(JointSensor &&) = default;
  JointSensor & operator=(JointSensor &&) = default;

  ~JointSensor() noexcept = default;

  /** Returns the sensor's joint name */
  inline const std::string & joint() const noexcept { return joint_; }

  /** Return the sensor's motor temperature reading (Celcius), NaN if not provided */
  inline double motorTemperature() const noexcept { return motor_temperature_; }

  /** Set the sensor's motor temperature reading (Celcius) */
  inline void motorTemperature(double motor_temperature) noexcept { motor_temperature_ = motor_temperature; }

  /** Return the sensor's driver temperature reading (Celcius), NaN if not provided */
  inline double driverTemperature() const noexcept { return driver_temperature_; }

  /** Set the sensor's driver temperature reading (Celcius) */
  inline void driverTemperature(double driver_temperature) noexcept { driver_temperature_ = driver_temperature; }

  /** Return the sensor's current reading (Ampere), NaN if not provided */
  inline double motorCurrent() const noexcept { return motor_current_; }

  /** Set the sensor's current reading (Ampere) */
  inline void motorCurrent(double motor_current) noexcept { motor_current_ = motor_current; }

  /** Return the sensor's motor ON/OFF reading, ON (true) if not provided */
  inline bool motorStatus() const noexcept { return motor_status_; }

  /** Set the sensor's motor ON/OFF reading */
  inline void motorStatus(bool motor_status) noexcept { motor_status_ = motor_status; }

protected:
  /** Name of joint to which sensor is attached */
  std::string joint_;

private:
  /* Motor temperature (Celcius) */
  double motor_temperature_ = std::numeric_limits<double>::quiet_NaN();
  /* Drive temperature (Celcius) */
  double driver_temperature_ = std::numeric_limits<double>::quiet_NaN();
  /* Motor current (Ampere) */
  double motor_current_ = std::numeric_limits<double>::quiet_NaN();
  /* Motor status (ON/OFF) */
  bool motor_status_ = true;
};

inline bool operator==(const JointSensor & lhs, const JointSensor & rhs)
{
  return lhs.joint() == rhs.joint();
}

} // namespace mc_rbdyn
