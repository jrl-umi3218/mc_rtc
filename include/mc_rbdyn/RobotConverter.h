/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotConverterConfig.h>

namespace mc_rbdyn
{

/**
 * @brief Copies all common properties from one robot to another
 * encoders, sensors, etc)
 *
 * @warning When using the RobotConverter to output to a real robot system, be
 * careful about your choice of options or RobotConverterConfig, not all
 * combinations of options are safe (e.g mbcToOutMbc=false and encodersToOutMbc=false would result in all joints going
 * to zero).
 */
struct MC_RBDYN_DLLAPI RobotConverter
{
  /** Initialize the converter and run it once on the output robot
   *
   * \param inputRobot Same robot that will be fed to \ref convert
   *
   * \param outputRobot Same robot that will be fed to \ref convert
   *
   * \param config Config used by this converter
   */
  RobotConverter(const mc_rbdyn::Robot & inputRobot,
                 mc_rbdyn::Robot & outputRobot,
                 const RobotConverterConfig & config);

  /**
   * @brief Copies the common properties of the input robot to the output robot
   *
   * See mc_rbdyn::RobotConverterConfig for available copy options.
   */
  void convert(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot) const;

  inline const RobotConverterConfig & config() const noexcept
  {
    return config_;
  }

protected:
  /**
   * @brief Precomputes the mappings of common properties between input/output
   * robot according to the chosen config_
   */
  void precompute(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & outputRobot);

  /**
   *
   * Copy encoders to output robot configuration
   *
   */
  void encodersToOutput(const mc_rbdyn::Robot & inputRobot, mc_rbdyn::Robot & outputRobot) const;

protected:
  RobotConverterConfig config_;
  // Common joint indices from inputRobot_ -> outputRobot_ robot
  std::vector<std::pair<unsigned int, unsigned int>> commonJointIndices_{};
  // Encoder indices from inputRobot_ -> outputRobot_ robot
  std::vector<std::pair<unsigned int, unsigned int>> commonEncoderToJointIndices_{};
  // Indices of joints with mimics from actuated joints in inputRobot_
  // to their mimic counterpart in outputRobot_
  std::vector<std::pair<unsigned int, unsigned int>> mimicJoints_{};
};
} // namespace mc_rbdyn
