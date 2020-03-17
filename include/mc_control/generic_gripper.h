/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_rbdyn/Mimic.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/constants.h>
#include <boost/math/constants/constants.hpp>
#include <algorithm>

#include <map>
#include <string>
#include <vector>

namespace mc_rtc
{
namespace gui
{
struct StateBuilder;
} // namespace gui
} // namespace mc_rtc

namespace mc_control
{

/*! \brief A robot's gripper reprensentation
 *
 * A gripper is composed of a set of joints that we want to control only
 * through "manual" operations. It may include passive joints.
 *
 * By default, a gripper is considered "open" when its joints' values
 * are at maximum value and "closed" at minimum value. This behaviour
 * can be reversed when the gripper is created.
 *
 * In real operations the actuated joints are also monitored to avoid
 * potential servo errors.
 */
struct MC_CONTROL_DLLAPI Gripper
{
  /*! Percentage of max velocity of active joints in the gripper */
  static constexpr double DEFAULT_PERCENT_VMAX = 0.25;
  /*! Difference between the command and the reality that triggers the safety */
  static constexpr double DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER = mc_rtc::constants::toRad(8.);
  /*! Number of iterations before the security is triggered */
  static constexpr unsigned int DEFAULT_OVER_COMMAND_LIMIT_ITER_N = 5;
  /** Release offset [rad] */
  static constexpr double DEFAULT_RELEASE_OFFSET = mc_rtc::constants::toRad(2);

  /*! \brief Constructor
   *
   * \param robot The full robot including uncontrolled joints
   * \param jointNames Name of the active joints involved in the gripper
   * \param robot_urdf URDF of the robot
   * \param currentQ Current values of the active joints involved in the gripper
   * \param timeStep Controller timestep
   * \param reverseLimits If set to true, then the gripper is considered "open" when the joints' values are minimal
   */
  Gripper(const mc_rbdyn::Robot & robot,
          const std::vector<std::string> & jointNames,
          const std::string & robot_urdf,
          const std::vector<double> & currentQ,
          double timeStep,
          bool reverseLimits = false);

  /*! \brief Constructor
   *
   * This constructor does not use information from the URDF file
   *
   * \param robot Robot, must have the active joints of the gripper to work properly
   * \param jointNames Name of the active joints involved in the gripper
   * \param mimics Mimic joints for the gripper
   * \param currentQ Current values of the active joints
   * \param timeStep Controller timestep
   * \param reverseLimits If true, the gripper is considered "open" when the joints values are minimal
   */
  Gripper(const mc_rbdyn::Robot & robot,
          const std::vector<std::string> & jointNames,
          const std::vector<mc_rbdyn::Mimic> & mimics,
          const std::vector<double> & currentQ,
          double timeStep,
          bool reverseLimits = false);

  /** \brief Resets the gripper parameters to their default value (percentVMax, actualCommandDiffTrigger) */
  void resetDefaults();

  /** \brief Saves the current gripper configuration parameters contained in
   * Config **/
  void saveConfig();

  /** \brief Restores the gripper configuration parameters from their saved
   * value
   *
   * \see saveConfig()
   **/
  void restoreConfig();

  /*! \brief Set the current configuration of the active joints involved in the gripper
   * \param curentQ Current values of the active joints involved in the gripper
   */
  void setCurrentQ(const std::vector<double> & currentQ);

  /*! \brief Set the target configuration of the active joints involved in the gripper
   * \param targetQ Desired values of the active joints involved in the gripper
   */
  void setTargetQ(const std::vector<double> & targetQ);

  /*! \brief Set the target opening of the gripper
   * \param targetOpening Opening value ranging from 0 (closed) to 1 (open)
   */
  void setTargetOpening(double targetOpening);

  /*! \brief Get current configuration
   * \return Current values of the active joints involved in the gripper
   */
  std::vector<double> curPosition() const;

  /*! \brief Return all gripper joints configuration
   * \return Current values of all the gripper's joints, including passive joints
   */
  const std::vector<double> & q();

  /*! \brief Get the current opening percentage
   *
   * \note Returns an average of the current opening percentage of each joint
   *
   * \return Current opening percentage
   */
  double opening() const;

  /*! \brief Set the encoder-based values of the gripper's active joints
   * \param q Encoder value of the gripper's active joints
   */
  void setActualQ(const std::vector<double> & q);

  /** Set gripper speed as a percentage of maximum velocity */
  void percentVMAX(double percent);
  /** Get gripper speed (percentage of max velocity) */
  double percentVMAX() const;

  /*! Set safety trigger threshold (difference between the command and the reality)
   *
   * This safety is meant to prevent over-torques on position controlled
   * grippers with no torque readings by checking how far the encoder output is
   * from the desired command. If it is over the limit, it can only stay there
   * for overCommandLimitIterN iterations before being released
   **/
  void actualCommandDiffTrigger(double d)
  {
    config_.actualCommandDiffTrigger = d;
  }
  /*! Difference between the command and the reality that triggers the safety */
  double actualCommandDiffTrigger() const
  {
    return config_.actualCommandDiffTrigger;
  }

  /*! Number of iterations where actualCommandDiffTrigger() threshold may be
   * exceeded before the security is triggered */
  void overCommandLimitIterN(unsigned int N)
  {
    config_.overCommandLimitIterN = std::max(N, 1u);
  }
  /*! Number of iterations where actualCommandDiffTrigger() threshold may be
   * exceeded before the security is triggered */
  unsigned int overCommandLimitIterN() const
  {
    return config_.overCommandLimitIterN;
  }

  /** Offset by which the gripper is released if overCommandDiffTrigger is
   * trigger for more than overCommandLimitIterN
   *
   * @param offset offset angle in [rad]
   **/
  void releaseSafetyOffset(double offset)
  {
    config_.releaseSafetyOffset = offset;
  }
  /** Offset by which the gripper is release if overCommandDiffTrigger is
   * trigger for more than overCommandLimitIterN */
  double releaseSafetyOffset() const
  {
    return config_.releaseSafetyOffset;
  }

  /*! \brief Check if the gripper motion stopped moving.
   *
   * The gripper will stop if
   * - the desired motion is finished
   * - the gripper encountered an obstacle and gripper safety was triggered.
   *   This is defined by an encoder error threshold (actualCommandDiffTrigger) and a maximum number of iterations
   *   where the gripper is allowed to be at this threshold (overCommandLimitIterN)
   *
   * \return True if gripper is not moving, False if it is moving
   */
  bool complete() const;

  /* Gripper gui will be added under {category, name} category */
  void addToGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category);
  /* Gripper gui will be removed from {category, name} category */
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category);

public:
  /*! Gripper name */
  std::string name;
  /*! Name of joints involved in the gripper */
  std::vector<std::string> names;
  /*! Name of active joints involved in the gripper */
  std::vector<std::string> active_joints;

  /*! Lower limits of active joints in the gripper (closed-gripper values) */
  std::vector<double> closeP;
  /*! Upper limits of active joints in the gripper (open-gripper values) */
  std::vector<double> openP;
  /*! Maximum velocity of active joints in the gripper */
  std::vector<double> vmax;

  /*! Mimic multiplier, first element is the joint to mimic, second is the multiplier */
  std::vector<std::pair<size_t, double>> mult;
  /*! Mimic offsets */
  std::vector<double> offset;
  /*! Full joints' values */
  std::vector<double> _q;

  /*! Current gripper target */
  std::vector<double> targetQIn;
  /*! Current gripper target: NULL if target has been reached or safety was triggered */
  std::vector<double> * targetQ;

  /*! Joints' values from the encoders */
  std::vector<double> actualQ;

protected:
  /** User-controllable parameters for the gripper */
  struct Config
  {
    /*! Percentage of max velocity of active joints in the gripper */
    double percentVMax = DEFAULT_PERCENT_VMAX;
    /*! Difference between the command and the reality that triggers the safety */
    double actualCommandDiffTrigger = DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER;
    /** Offset by which the gripper is released when safety is triggered */
    double releaseSafetyOffset = DEFAULT_RELEASE_OFFSET;
    /*! Number of iterations before the security is triggered */
    unsigned int overCommandLimitIterN = DEFAULT_OVER_COMMAND_LIMIT_ITER_N;
  };
  /** Current configuration of the gripper parameters */
  Config config_;
  /** Saved configuration of the parameters saved by savedConfig() */
  Config savedConfig_;

  /*! Current opening percentage */
  std::vector<double> percentOpen;
  /*! Controller timestep */
  double timeStep = 0;
  /*! True if the gripper has been too far from the command for over overCommandLimitIterN iterations */
  std::vector<bool> overCommandLimit;
  /*! Store the number of iterations where the gripper command was over the limit */
  std::vector<unsigned int> overCommandLimitIter;
  /*! True if the gripper is reversed */
  bool reversed = false;
};

} // namespace mc_control
