/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/generic_gripper.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rbdyn/Flexibility.h>
#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/JointSensor.h>
#include <mc_rbdyn/Springs.h>

#include <memory>
#include <vector>

namespace mc_rbdyn
{

struct Robot;

/** Hold data and objects that are shared among different views of a robot (control/real/output)
 *
 * The framework enforce consistency between these views
 *
 * This includes:
 * - reference joint order
 * - encoder level readings (position/velocity/torques)
 * - force/body/joint sensors
 * - grippers
 * - devices
 */
struct RobotData
{
  /** Reference joint order see mc_rbdyn::RobotModule */
  std::vector<std::string> refJointOrder;
  /** Encoders' positions provided in the robot's ref joint order */
  std::vector<double> encoderValues;
  /** Encoders' velocities provided in the robot's ref joint order */
  std::vector<double> encoderVelocities;
  /** Joint torques provided by the low-level controller */
  std::vector<double> jointTorques;
  /* Force sensors */
  std::vector<ForceSensor> forceSensors;
  /** Correspondance between force sensor's name and force sensor index */
  std::unordered_map<std::string, size_t> forceSensorsIndex;
  /** Correspondance between bodies' names and attached force sensors */
  std::unordered_map<std::string, size_t> bodyForceSensors_;
  /** Hold all body sensors */
  BodySensorVector bodySensors;
  /** Correspondance between body sensor's name and body sensor index*/
  std::unordered_map<std::string, size_t> bodySensorsIndex;
  /** Correspondance between bodies' names and attached body sensors */
  std::unordered_map<std::string, size_t> bodyBodySensors;
  /** Hold all joint sensors */
  std::vector<JointSensor> jointSensors;
  /** Correspondance between joints' names and attached joint sensors */
  std::unordered_map<std::string, size_t> jointJointSensors;
  /** Grippers attached to this robot (empty until the robot is loaded into a controller) */
  std::unordered_map<std::string, mc_control::GripperPtr> grippers;
  /** Grippers reference for this robot */
  std::vector<mc_control::GripperRef> grippersRef;
  /** Hold all devices that are neither force sensors nor body sensors */
  DevicePtrVector devices;
  /** Correspondance between a device's name and a device index */
  std::unordered_map<std::string, size_t> devicesIndex;
  /** A list of robots that share this data
   *
   * This is used to communicate changes to the data to all instances that share this data */
  std::vector<Robot *> robots;
};

using RobotDataPtr = std::shared_ptr<RobotData>;

} // namespace mc_rbdyn
