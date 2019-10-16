/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>
#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_tasks
{

/*! \brief Generic CRTP implementation for a task tracking a curve in both position
 * and orientation. This class takes care of much of the logic behind tracking a
 * curve:
 * - Interpolates between orientation waypoints defined as pairs [time, orientation]
 * - Provides reference position/velocity/acceleration from underlying curve
 *   (defined in the Derived class)
 * - Implements GUI elements for controlling waypoints, targets, etc.
 * - Brings in all the functionalities from TrajectoryTaskGeneric
 *
 * To implement tracking of a new curve, simply derive from this class and
 * implement the functionalites specific to the curve. You need to at least
 * implement:
 * - spline() accessors
 * - target() accessors/setters
 * The spline itself (as returned by spline()) should at least implement:
 * - update() triggers the curve computation if needed (if waypoints/constraints
 *   changed)
 * - target() accessors/setters
 * - splev(t, order) computes the position and its nth-order derivatives at time t along
 *   the curve
 */
template<typename Derived>
struct SplineTrajectoryTask : public TrajectoryTaskGeneric<tasks::qp::TransformTask>
{
  using SplineTrajectoryBase = SplineTrajectoryTask<Derived>;
  using TrajectoryTask = TrajectoryTaskGeneric<tasks::qp::TransformTask>;

  /*! \brief Constructor
   *
   * \param robots Robots controlled by the task
   * \param robotIndex Which robot is controlled
   * \param surfaceName Surface controlled by the task (should belong to the controlled robot)
   * \param duration Length of the movement
   * \param stiffness Task stiffness (position and orientation)
   * \param posW Task weight (position)
   * \param oriW Task weight (orientation)
   * \param oriWp Optional orientation waypoints (pairs of [time, orientation])
   */
  SplineTrajectoryTask(const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       const std::string & surfaceName,
                       double duration,
                       double stiffness,
                       double weight,
                       const Eigen::Matrix3d & target,
                       const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

  /** Add support for the following entries
   *
   * - timeElapsed When true, the task will stop when the trajectory duration is
   *   reached
   */
  std::function<bool(const mc_tasks::MetaTask &, std::string &)> buildCompletionCriteria(
      double dt,
      const mc_rtc::Configuration & config) const override;

  /** \brief Sets the orientation waypoints
   *
   * \param oriWp Orientation waypoints defined as pairs of [time, orientation]
   */
  void oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);

  /*! \brief Sets the dimensional weights (controls the importance of
   * orientation/translation).
   *
   * \throw if dimW is not a Vector6d
   *
   * \param dimW Weights expressed as a Vector6d [wx, wy, wz, tx, ty, tz]
   */
  void dimWeight(const Eigen::VectorXd & dimW) override;

  /*! \brief Gets the dimensional weights (orientation/translation)
   *
   * \returns Dimensional weights expressed as a Vector6d [wx, wy, wz, tx, ty, tz]
   */
  Eigen::VectorXd dimWeight() const override;

  /*! \brief Whether the trajectory has finished
   *
   * \returns True if the trajectory has finished
   */
  bool timeElapsed() const;

  /*! \brief Returns the transformError between current robot surface pose and
   * its final target
   *
   * \returns The error w.r.t the final target
   */
  Eigen::VectorXd eval() const override;

  /**
   * \brief Returns the trajectory tracking error: transformError between the current robot surface pose
   * and its next desired pose along the trajectory error
   *
   * \return The trajectory tracking error
   */
  virtual Eigen::VectorXd evalTracking() const;

  /*! \brief Sets the curve target pose.
   * Translation target will be handled by the Derived curve, while orientation
   * target is interpolated here.
   *
   * \param target Target pose for the curve
   */
  void target(const sva::PTransformd & target);
  /*! \brief Gets the target pose (position/orientation)
   *
   * \returns target pose
   */
  const sva::PTransformd target() const;

  /*! \brief Get the control points of the trajectory's b-spline
   *
   * \returns The list of control points in the spline
   */
  std::vector<Eigen::Vector3d> controlPoints();

  /*! \brief Number of points to sample on the spline for the gui display
   *
   * \param s number of samples
   */
  void displaySamples(unsigned s);
  /*! \brief Number of samples for displaying the spline
   *
   * \return number of samples
   */
  unsigned displaySamples() const;

protected:
  /**
   * \brief Tracks a reference world pose
   *
   * \param pose Desired position (world)
   */
  void refPose(const sva::PTransformd & pose);
  /**
   * \brief Returns the trajectory reference world pose
   *
   * \return Desired pose (world)
   */
  const sva::PTransformd & refPose() const;

  /* Hide parent's refVel and refAccel implementation as the Spline tasks
   * are overriding these at every iteration according to the underlying curve */
  using TrajectoryTaskGeneric<tasks::qp::TransformTask>::refVel;
  using TrajectoryTaskGeneric<tasks::qp::TransformTask>::refAccel;

  /*! \brief Add task controls to the GUI.
   * Interactive controls for the trajectory waypoints and end-endpoints
   * automatically updates the curve
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  /*! \brief Add the task to a solver
   * \param solver Solver where to add the task
   */
  void addToSolver(mc_solver::QPSolver & solver) override;

  /*! \brief Add information about the task to the logger
   *
   * @param logger
   */
  void addToLogger(mc_rtc::Logger & logger) override;

  void removeFromLogger(mc_rtc::Logger & logger) override;

  /*! \brief Update trajectory target
   */
  void update() override;

protected:
  unsigned int rIndex_;
  std::string surfaceName_;
  double duration_;
  mc_trajectory::InterpolatedRotation oriSpline_;
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp_;

  double currTime_ = 0.;
  double timeStep_ = 0;
  unsigned samples_ = 20;
  bool inSolver_ = false;
};
} // namespace mc_tasks

#include <mc_tasks/SplineTrajectoryTask.hpp>
