/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/api.h>
#include <mc_rbdyn/Mimic.h>
#include <mc_rbdyn/Robots.h>

#include <map>
#include <string>
#include <vector>

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
public:
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

public:
  /*! Gripper name */
  std::string name;
  /*! Name of joints involved in the gripper */
  std::vector<std::string> names;
  /*! Name of active joints involved in the gripper */
  std::vector<std::string> active_joints;

  /*! True if the gripper is reversed */
  bool reversed;
  /*! Lower limits of active joints in the gripper (closed-gripper values) */
  std::vector<double> closeP;
  /*! Upper limits of active joints in the gripper (open-gripper values) */
  std::vector<double> openP;
  /*! Maximum velocity of active joints in the gripper */
  std::vector<double> vmax;
  /*! Controller timestep */
  double timeStep;
  /*! Current opening percentage */
  std::vector<double> percentOpen;

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

  /*! True if the gripper has been too far from the command for over overCommandLimitIterN iterations */
  std::vector<bool> overCommandLimit;
  /*! Store the number of iterations where the gripper command was over the limit */
  std::vector<unsigned int> overCommandLimitIter;
  /*! Number of iterations before the security is triggered */
  unsigned int overCommandLimitIterN;
  /*! Joints' values from the encoders */
  std::vector<double> actualQ;
  /*! Difference between the command and the reality that triggers the safety */
  double actualCommandDiffTrigger;
};

} // namespace mc_control
