/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Device.h>

namespace mc_rbdyn
{

struct Robot;

/** This struct is intended to hold static information about an external torques sensor
 * and the current reading of said sensor.
 */
struct MC_RBDYN_DLLAPI VirtualTorqueSensor : public Device
{
public:
  /** Default constructor, this does not represent a valid external torques sensor */
  VirtualTorqueSensor();

  /** Construct a valid external torques sensor based on static information, this
   * force sensor can then be used to provide sensing information to the robot.
   *
   * @param name Name of the sensor
   *
   * @param size Number of DoF
   *
   */
  VirtualTorqueSensor(const std::string & name, const int & size);

  VirtualTorqueSensor(const VirtualTorqueSensor & ets);

  VirtualTorqueSensor & operator=(const VirtualTorqueSensor & ets);

  VirtualTorqueSensor(VirtualTorqueSensor &&) = default;

  /** Destructor */
  ~VirtualTorqueSensor() noexcept override;

  inline const Eigen::VectorXd & torques() const { return virtualJointTorques_; }

  inline void torques(Eigen::VectorXd & torques) { virtualJointTorques_ = torques; }

  DevicePtr clone() const override;

private:
  Eigen::VectorXd virtualJointTorques_;
  int size_;
};

} // namespace mc_rbdyn
