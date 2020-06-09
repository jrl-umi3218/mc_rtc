/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/SplineTrajectoryTask.h>
#include <mc_trajectory/ExactCubic.h>

namespace mc_tasks
{

/*! \brief Track an exact cubic spline, that is a curve passing exactly through
 * waypoints in position, with optional initial and final velocity and
 * acceleration.
 *
 * SplineTrajectoryTask takes care of much of the logic (task target updates,
 * orientation waypoint handling, etc.), and brings in all the functionalities
 * from TrajectoryTaskGeneric. This ExactCubicTrajectoryTask only handles specific
 * aspect of the ExactCubic curve.
 */
struct MC_TASKS_DLLAPI ExactCubicTrajectoryTask : public SplineTrajectoryTask<ExactCubicTrajectoryTask>
{
  friend struct SplineTrajectoryTask<ExactCubicTrajectoryTask>;

  /**
   * \brief Trajectory following an exact cubic spline with given initial and
   * final acceleration/velocity. The curve will pass exactly through the
   * specified position waypoints (see posWaypoints) and will interpolate
   * between orientation waypoints. Initial and final acceleration/velocity will
   * also be enforced.
   *
   * \param robots Robots controlled by the task
   * \param robotIndex  Which robot is controlled
   * \param surfaceName Surface controlled by the task, should belong to the
   * \ontrolled robot
   * \param duration Duration of motion (eg time it takes to go from the current
   * \urface position to the curve's final point)
   * \param stiffness Task stiffness
   * \param weight Task weight
   * \param target Final world pose to reach
   * \param posWp Waypoints in position specified as pairs of [time, position]
   * \param init_vel Initial velocity of the curve (default: Zero)
   * \param init_acc Initial acceleration of the curve (default: Zero)
   * \param end_vel Final velocity of the curve (default: Zero)
   * \param enc_acc Final acceleration of the curve (default: Zero)
   * \param oriWp Waypoints in orientation, specified as pairs of [time,
   * orientation]. Orientation is interpolated in between waypoints. (default :
   * none)
   */
  ExactCubicTrajectoryTask(const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           const std::string & surfaceName,
                           double duration,
                           double stiffness,
                           double weight,
                           const sva::PTransformd & target,
                           const std::vector<std::pair<double, Eigen::Vector3d>> & posWp = {},
                           const Eigen::Vector3d & init_vel = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & init_acc = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & end_vel = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & end_acc = Eigen::Vector3d::Zero(),
                           const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

  /*! \brief const accessor to the underlying spline (used by SplineTrajectoryTask)
   *
   * \returns The spline
   */
  const mc_trajectory::ExactCubic & spline() const
  {
    return bspline;
  };
  /*! \brief accessor to the underlying spline (used by SplineTrajectoryTask)
   *
   * \returns The spline
   */
  mc_trajectory::ExactCubic & spline()
  {
    return bspline;
  };

  /*! \brief Add interactive GUI elements to control the curve waypoints
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  /** \brief Waypoints in position. The curve will pass exactly through these waypoints.
   */
  void posWaypoints(const std::vector<std::pair<double, Eigen::Vector3d>> & posWp);

  /** \brief Initial and final velocity and acceleration constraints for the
   * curve
   *
   * \param init_vel Initial velocity of the curve (default: Zero)
   * \param init_acc Initial acceleration of the curve (default: Zero)
   * \param end_vel Final velocity of the curve (default: Zero)
   * \param enc_acc Final acceleration of the curve (default: Zero)
   */
  void constraints(const Eigen::Vector3d & init_vel,
                   const Eigen::Vector3d & init_acc,
                   const Eigen::Vector3d & end_vel,
                   const Eigen::Vector3d & end_acc);

protected:
  /*! \brief Sets the curve target pose
   * \param target Target pose for the curve
   */
  void targetPos(const Eigen::Vector3d & target);

  /** \brief Returns the curve's target position */
  const Eigen::Vector3d & targetPos() const;

protected:
  mc_trajectory::ExactCubic bspline;
  sva::PTransformd initialPose_;
};

} // namespace mc_tasks
