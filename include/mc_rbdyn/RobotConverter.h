/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/api.h>

namespace mc_rbdyn
{

/**
 * @brief Configuration for the RobotConverter
 *
 */
struct RobotConverterConfig
{
  ///< Copy input robot's mbc to their corresponding output robot's mbc
  bool mbcToOutMbc = true;
  /**
   * Chosse which mbc properties to copy, only effective if mbcToOutMbc = true
   * @{
   */
  bool q = true;
  bool alpha = true;
  bool alphad = true;
  bool tau = true;
  ///< @}

  ///< Copy input encoder values to the output robot's mbc
  bool encodersToOutMbc = true;
  ///< Copy input encoder values to the output robot's encoder values
  bool copyEncoderValues = true;
  ///< Copy input encoder velocities to the output robot's encoder velocities
  bool copyEncoderVelocities = true;
  ///< Copy input force sensors to the output robot's force sensors
  bool copyForceSensors = true;
  ///< Copy input body sensors to the output robot's body sensors
  bool copyBodySensors = true;
  ///< Compute the output robot's joint mimics
  bool computeMimics = true;
};

/**
 * @brief Copies all common properties from one robot to another (joint values,
 * encoders, sensors, etc)
 *
 * @warning When using the RobotConverter to output to a real robot system, be
 * careful about your choice of options or RobotConverterConfig, not all
 * combinations of options are safe (e.g mbcToOutMbc=false and encodersToOutMbc=false would result in all joints going
 * to zero).
 */
struct MC_RBDYN_DLLAPI RobotConverter
{
  RobotConverter(const RobotConverterConfig & config = RobotConverterConfig{});

  /**
   * @brief Copies the common properties of the input robot to the output robot
   *
   * See mc_rbdyn::RobotConverterConfig for available copy options.
   */
  void convert(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot);

  inline const RobotConverterConfig & config() const noexcept
  {
    return config_;
  }

protected:
  /**
   * @brief Precomputes the mappings of common properties between input/output
   * robot according to the chosen config_
   */
  void precompute(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot);

protected:
  RobotConverterConfig config_;
  bool first_ = true;
  // Common joint indices from inputRobot_ -> outputRobot_ robot
  std::vector<std::pair<unsigned int, unsigned int>> commonJointIndices_{};
  // Common encoder indices from inputRobot_ -> outputRobot_ robot
  std::vector<std::pair<unsigned int, unsigned int>> commonEncoderToJointIndices_{};
  // Indices from inputRobot_ encoders to outputRobot_ encoders
  std::vector<std::pair<unsigned int, unsigned int>> commonEncoderIndices_{};
  // Output encoder values
  std::vector<double> outEncoderValues_;
  // Output encoder velocities
  std::vector<double> outEncoderVelocities_;
  // Indices of joints with mimics from actuated joints in inputRobot_
  // to their mimic counterpart in outputRobot_
  std::vector<std::pair<unsigned int, unsigned int>> mimicJoints_{};
};
} // namespace mc_rbdyn
