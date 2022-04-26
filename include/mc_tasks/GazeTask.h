/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the Gaze of a body
 *
 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI GazeTask : public TrajectoryTaskGeneric<tasks::qp::GazeTask>
{
public:
  /*! \brief Constructor
   *
   * \param frame Control frame that should be attached to the camera frame where estimates are provided
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   * \param error Initial error value, error.z() must be > 0
   */
  GazeTask(const mc_rbdyn::RobotFrame & frame,
           double stiffness = 2.0,
           double weight = 500.0,
           const Eigen::Vector3d & error = Eigen::Vector3d::UnitZ());

  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param point2d Position of the point in image frame
   *
   * \param depthEstimate Distance of the monitored point from the camera
   *
   * \param X_b_gaze Transformation between the camera link and the parent body
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  GazeTask(const std::string & bodyName,
           const Eigen::Vector2d & point2d,
           double depthEstimate,
           const sva::PTransformd & X_b_gaze,
           const mc_rbdyn::Robots & robots,
           unsigned int robotIndex,
           double stiffness = 2.0,
           double weight = 500);

  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param point3d Equal to (point2d[0], point2d[1], depthEstimate).
   * Depth estimate must be >0, the constructor will throw otherwise as this
   * would result in a division by zero.
   *
   * \param X_b_gaze Transformation between the camera link and the parent body
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   * \throws std::logic_error if point3d.z() <= 0
   *
   */
  GazeTask(const std::string & bodyName,
           const Eigen::Vector3d & point3d,
           const sva::PTransformd & X_b_gaze,
           const mc_rbdyn::Robots & robots,
           unsigned int robotIndex,
           double stiffness = 2.0,
           double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task objective to the current body orientation
   */
  void reset() override;

  /*! \brief Set the current error
   *
   * \param point2d Target position in camera frame
   *
   * \param point2d_ref Desired position of the point in image frame
   *
   */
  void error(const Eigen::Vector2d & point2d, const Eigen::Vector2d & point2d_ref = Eigen::Vector2d::Zero());

  /*! \brief Set the current error
   *
   * \param point3d Target position in camera frame
   *
   * \param point2d_ref Desired position of the point in image frame
   *
   */
  void error(const Eigen::Vector3d & point3d, const Eigen::Vector2d & point2d_ref = Eigen::Vector2d::Zero());
};

} // namespace mc_tasks
