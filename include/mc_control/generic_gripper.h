/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_rbdyn/RobotModule.h>

#include <map>
#include <string>
#include <vector>

namespace mc_rbdyn
{
struct Robot;
} // namespace mc_rbdyn

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
struct MC_RBDYN_DLLAPI Gripper
{

  /*! \brief Constructor
   *
   * \param robot The full robot including uncontrolled joints
   * \param jointNames Name of the active joints involved in the gripper
   * \param robot_urdf URDF of the robot
   * \param reverseLimits If set to true, then the gripper is considered "open" when the joints' values are minimal
   * \param safety Default gripper safety parameters
   */
  Gripper(const mc_rbdyn::Robot & robot,
          const std::vector<std::string> & jointNames,
          const std::string & robot_urdf,
          bool reverseLimits,
          const mc_rbdyn::RobotModule::Gripper::Safety & safety);

  /*! \brief Constructor
   *
   * This constructor does not use information from the URDF file
   *
   * \param robot Robot, must have the active joints of the gripper to work properly
   * \param jointNames Name of the active joints involved in the gripper
   * \param mimics Mimic joints for the gripper
   * \param reverseLimits If true, the gripper is considered "open" when the joints values are minimal
   * \param safety Default gripper safety parameters
   */
  Gripper(const mc_rbdyn::Robot & robot,
          const std::vector<std::string> & jointNames,
          const std::vector<mc_rbdyn::Mimic> & mimics,
          bool reverseLimits,
          const mc_rbdyn::RobotModule::Gripper::Safety & safety);

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

  /** \brief Applies a new gripper configuration (safeties and targets) */
  void configure(const mc_rtc::Configuration & config);

  /*! \brief Reset the gripper state to the current actual state of the gripper
   *
   * \param currentQ Current encoder values for the robot
   */
  void reset(const std::vector<double> & currentQ);

  /*! \brief Reset from another gripper
   *
   * \param gripper Gripper used to reset this one
   */
  void reset(const Gripper & gripper);

  /*! \brief Run one iteration of control
   *
   * \param robot Robot for which this gripper control is running
   *
   * \param qOut Output of the gripper state
   *
   * The gripper control updates both the robot's configuration and the output
   */
  void run(double timeStep, mc_rbdyn::Robot & robot, std::map<std::string, std::vector<double>> & qOut);

  /*! \brief Set the target configuration of the active joints involved in the gripper
   * \param targetQ Desired values of the active joints involved in the gripper
   * \throw std::runtime_error If the targetQ size does not match the number of active joints
   */
  void setTargetQ(const std::vector<double> & targetQ);

  /*! \brief Set the target configuration of the specified active joint
   * \param jointName Name of the gripper's active joint to move
   * \param targetQ Desired value of the active joint
   * \throw std::runtime_error if the joint name does not match any of the
   * gripper's active joints
   */
  void setTargetQ(const std::string & jointName, double targetQ);

  /** Get a joint's target angle
   *
   * \throw std::runtime_error if the joint name does not match any of the
   * gripper's active joints
   */
  double getTargetQ(const std::string & jointName) const;

  /** Get the current gripper's target
   * \note returns the current gripper position if no target has been set
   * */
  std::vector<double> getTargetQ() const;

  /*! \brief Set the target opening of all gripper joints simultaneously
   * \param targetOpening Opening value ranging from 0 (closed) to 1 (open)
   *
   * \note If the individual joint targets were set manually, they will move to
   * match this new opening target. Depending on the maximum allowed velocity, this may result
   * in fast gripper joint motions. Use with care if the gripper is currently
   * grasping objects or close to collisions with the environment.
   */
  void setTargetOpening(double targetOpening);

  /*! \brief Set the target opening of a single gripper joint
   * \param jointName Name of the active joint to move
   * \param targetOpening Opening value ranging from 0 (closed) to 1 (open)
   * \throw std::runtime_error if the joint name does not match any of the
   * gripper's active joints
   */
  void setTargetOpening(const std::string & jointName, double targetOpening);

  /** \brief Get the target opening of a single gripper joint
   * \param jointName Name of the active joint
   * \throw std::runtime_error if the joint name does not match any of the
   * gripper's active joints
   * \return The joint's target opening percentage
   */
  double getTargetOpening(const std::string & jointName) const;

  /*! \brief Get current configuration
   * \return Current values of the active joints involved in the gripper
   */
  std::vector<double> curPosition() const;

  /*! \brief Get current opening percentage
   * \return Current opening percentage of the active joints involved in the gripper
   */
  std::vector<double> curOpening() const;

  /** \brief Get the current opening of a single gripper joint
   * \param jointName Name of the active joint
   * \throw std::runtime_error if the joint name does not match any of the
   * gripper's active joints
   * \return The joint's current opening percentage
   */
  double curOpening(const std::string & jointName) const;

  /*! \brief Returns all joints involved in the gripper */
  inline const std::vector<std::string> & joints() const
  {
    return names;
  }

  /*! \brief Returns all active joints involved in the gripper */
  inline const std::vector<std::string> & activeJoints() const
  {
    return active_joints;
  }

  /* \brief Checks whether a joint is an active gripper joint */
  inline bool hasActiveJoint(const std::string & jointName) const
  {
    return std::find(active_joints.begin(), active_joints.end(), jointName) != active_joints.end();
  }

  /*! \brief Return all gripper joints configuration
   * \return Current values of all the gripper's joints, including passive joints
   */
  inline const std::vector<double> & q() const
  {
    return _q;
  }

  /*! \brief Get the current opening percentage
   *
   * \note Returns an average of the current opening percentage of each joint
   *
   * \return Current opening percentage
   */
  double opening() const;

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
   * @param offset offset angle in [rad] or distance in [meter]
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

  /*! \brief Returns true if a gripper is metric, i.e. all active joints are prismatic rather than revolute
   *
   * This only affects safety settings in the GUI
   */
  inline bool is_metric() const noexcept
  {
    return is_metric_;
  }

protected:
  /*! \brief Set the target opening of a single gripper joint by index
   * \param activeJointId Index of the active joint
   * \param targetOpening Opening value ranging from 0 (closed) to 1 (open)
   */
  void setTargetOpening(size_t activeJointId, double targetOpening);

  /*! \brief Set the target configuration of the specified active joint
   * \param activeJointId Index of the gripper's active joint to move
   * \param targetQ Desired value of the active joint. Clamps the target within
   * the joint limits.
   */
  void setTargetQ(size_t activeJointId, double targetQ);

  /// Fast version of setTargetQ(size_t, double) without clamping
  void setTargetQ_(size_t activeJoints, double targetQ);
  /// Fast version of setTargetQ(std::vector<double>)
  void setTargetQ_(const std::vector<double> & targetQ);

  /// Current position of an active joint
  double curPosition(size_t jointId) const;

  /// Current opening percentage of an active joint
  double curOpening(size_t jointId) const;

  /// Target opening percentage of an active joint
  double targetOpening(size_t jointId) const;

  /// Target opening angle of an active joint
  double getTargetQ(size_t jointId) const;

  /// Clamp an active joint value within its joint limits
  double clampQ(size_t activeJoint, double q);

protected:
  /*! Name of joints involved in the gripper */
  std::vector<std::string> names;
  /*! Name of active joints involved in the gripper */
  std::vector<std::string> active_joints;
  /*! Active joints indexes in the reference joint order */
  std::vector<size_t> active_joints_idx;
  /*! All joint indexes in mbc, -1 if absent */
  std::vector<int> joints_mbc_idx;

  /*! True if all joints are primatic */
  bool is_metric_;

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

  /*! Target reached threshold per-joint (0.001 rad for revolute and 0.0001 for prismatic joints) */
  std::vector<double> reached_threshold_;

  /*! Joints' values from the encoders */
  std::vector<double> actualQ;

protected:
  using Config = mc_rbdyn::RobotModule::Gripper::Safety;
  /** Current configuration of the gripper parameters */
  Config config_;
  /** Saved configuration of the parameters saved by saveConfig() */
  Config savedConfig_;
  /** Default configuration provided at construction */
  Config defaultConfig_;

  /*! Current opening percentage */
  std::vector<double> percentOpen;
  /*! True if the gripper has been too far from the command for over overCommandLimitIterN iterations */
  std::vector<bool> overCommandLimit;
  /*! Store the number of iterations where the gripper command was over the limit */
  std::vector<unsigned int> overCommandLimitIter;
  /*! True if the gripper is reversed */
  bool reversed = false;
};

using GripperPtr = std::unique_ptr<Gripper>;
using GripperRef = std::reference_wrapper<Gripper>;

} // namespace mc_control
