/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>
#include <mc_trajectory/InterpolatedRotation.h>
#include <mc_trajectory/SequenceInterpolator.h>

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
 * - targetPos() accessors/setters
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
  using SequenceInterpolator6d =
      mc_trajectory::SequenceInterpolator<Eigen::Vector6d, mc_trajectory::LinearInterpolation<Eigen::Vector6d>>;

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

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

  /** Add support for the following entries
   *
   * - timeElapsed: True when the task duration has elapsed
   * - wrench: True when the force applied on the robot surface is higher than the provided threshold (6d vector, NaN
   * value ignores the reading, negative values invert the condition). Ignored if the surface has no force-sensor
   * attached.
   *   @throws if the surface does not have an associated force sensor
   */
  std::function<bool(const mc_tasks::MetaTask &, std::string &)> buildCompletionCriteria(
      double dt,
      const mc_rtc::Configuration & config) const override;

  /** \brief Sets the orientation waypoints
   *
   * \param oriWp Orientation waypoints defined as pairs of [time, orientation]
   */
  void oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);

  /** @name Setters for the tasks gains and gain interpolation (dimWeight/Stiffnes/Damping)
   *
   * This task supports two different ways to define gains:
   * - the \ref dimWeight, \ref stiffness, \ref damping and \ref setGains setters inherited from TrajectoryTaskGeneric
   * - linear interpolation of gains values managed by \ref dimWeightInterpolation, \ref stiffnessInterpolation, \ref
   * dampingInterpolation
   *
   * If an interpolation method is present, calling the corresponding dimWeight/stiffness/damping/setGains setter will
   * disable interpolation and set a constant gain. The interpolation setters ensure that gains are interpolated
   * starting from the current (time, gain) values.
   *
   * @{
   */

  // Ensures that accessors from the base class are available
  using TrajectoryBase::damping;
  using TrajectoryBase::dimWeight;
  using TrajectoryBase::stiffness;

  /*! \brief Sets the dimensional weights (controls the importance of
   * orientation/translation).
   *
   * \note Removes the dimWeightInterpolator_ if it exists
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

  /*! \brief Set the task stiffness/damping
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \note Removes the stiffnessInterpolator_ if it exists
   *
   * \param stiffness Task stiffness
   */
  void stiffness(double stiffness);

  /*!
   * \anchor stiffness
   * \brief Set dimensional stiffness
   *
   * The caller should be sure that the dimension of the vector fits the task dimension.
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \note Removes the stiffnessInterpolator_ if it exists
   *
   * \param stiffness Dimensional stiffness
   */
  void stiffness(const Eigen::VectorXd & stiffness);

  /*! \brief Set the task damping, leaving its stiffness unchanged
   *
   * \note Removes the dampingInterpolator_ if it exists
   *
   * \param damping Task stiffness
   */
  void damping(double damping);

  /*!
   * \anchor damping
   * \brief Set dimensional damping
   *
   * The caller should be sure that the dimension of the vector fits the task dimension.
   *
   * \note Removes the dampingInterpolator_ if it exists
   *
   * \param damping Dimensional damping
   */
  void damping(const Eigen::VectorXd & damping);

  /*! \brief Set both stiffness and damping
   *
   * \param stiffness Task stiffness
   *
   * \note Removes the stiffnessInterpolator_ and dampingInterpolator_ if they exist
   *
   * \param damping Task damping
   */
  void setGains(double stiffness, double damping);

  /*!
   * \anchor setGains
   *
   * \brief Set dimensional stiffness and damping
   *
   * The caller should be sure that the dimensions of the vectors fit the task dimension.
   *
   * \note Removes the stiffnessInterpolator_ and dampingInterpolator_ if they exist
   *
   * \param stiffness Dimensional stiffness
   *
   * \param damping Dimensional damping
   */
  void setGains(const Eigen::VectorXd & stiffness, const Eigen::VectorXd & damping);

  /**
   * \anchor dimWeightInterpolation
   *
   * Interpolate the dimensional weight between the specified values
   *
   * \param dimWeights Pairs of [time, dimensional gain], strictly ordered by ascending time.
   *
   * \note To remove interpolation, call \ref dimWeight
   * \note If you call this function after the task is started, and wish to keep the gains continuous, you should insert
   * the [currentTime(), dimWeight()] pair in the dimWeights argument.
   *
   * \throws std::runtime_error If dimWeights is not meeting the requirements
   */
  void dimWeightInterpolation(const std::vector<std::pair<double, Eigen::Vector6d>> & dimWeights);
  inline const std::vector<std::pair<double, Eigen::Vector6d>> & dimWeightInterpolation() const noexcept
  {
    return dimWeightInterpolator_.values();
  }

  /**
   * \anchor stiffnessInterpolation
   *
   * Interpolate the stiffness between the provided values
   *
   * \param stiffnessGains Pairs of [time, dimensional gain], strictly ordered by ascending time.
   *
   * \note If there is no \ref dampingInterpolation, damping will be set to 2*sqrt(stiffness)
   * \note To remove interpolation, call one of the stiffness setters (see \ref stiffness)
   * \note If you call this function after the task is started, and wish to keep the gains continuous, you should insert
   * the [currentTime(), dimWeight()] pair in the stiffnessGains argument.
   *
   * \throws std::runtime_error If stiffnessGains is not meeting the requirements
   */
  void stiffnessInterpolation(const std::vector<std::pair<double, Eigen::Vector6d>> & stiffnessGains);
  inline const std::vector<std::pair<double, Eigen::Vector6d>> & stiffnessInterpolation() const noexcept
  {
    return stiffnessInterpolator_.values();
  }

  /**
   * \anchor dampingInterpolation
   * Interpolate the dimensional damping between the provided values
   *
   * \param dampingGains Pairs of [time, dimensional gain]. Must be strictly
   * ordered by ascending time, and time > currentTime()
   *
   * \note To remove interpolation, call one of the damping setters (see \ref damping)
   * \note If you call this function after the task is started, and wish to keep the gains continuous, you should insert
   * the [currentTime(), dimWeight()] pair in the dampingGains argument.
   *
   * \throws std::runtime_error If dampingGains is not meeting the requirements
   */
  void dampingInterpolation(const std::vector<std::pair<double, Eigen::Vector6d>> & dampingGains);
  inline const std::vector<std::pair<double, Eigen::Vector6d>> & dampingInterpolation() const noexcept
  {
    return dampingInterpolator_.values();
  }

  /** @} */

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

  /**
   * @brief Allows to pause the task
   *
   * This feature is mainly intended to allow starting the task in paused state
   * to allow adjusting the parameters of the trajectory before its execution.
   * Use with care in other contexts.
   *
   * @warning Pausing sets the task's desired velocity and acceleration to zero, which
   * will suddently stop the motion. Unpausing causes the motion to resume
   * at the current speed along the trajectory.
   * Avoid pausing/resuming during high-speed trajectories.
   *
   * @param paused True to pause the task, False to resume.
   */
  inline void pause(bool paused)
  {
    paused_ = paused;
  }

  /** True when the task is paused */
  inline bool pause() const noexcept
  {
    return paused_;
  }

  /** Returns the current time along the trajectory */
  inline double currentTime() const noexcept
  {
    return currTime_;
  }

  /** @brief Returns the trajectory's duration */
  inline double duration() const noexcept
  {
    return duration_;
  }

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

  /*! \brief Add information about the task to the logger
   *
   * @param logger
   */
  void addToLogger(mc_rtc::Logger & logger) override;

  void removeFromLogger(mc_rtc::Logger & logger) override;

  /*! \brief Update trajectory target */
  void update(mc_solver::QPSolver &) override;

  /** Interpolate dimWeight, stiffness, damping */
  void interpolateGains();

protected:
  unsigned int rIndex_ = 0;
  std::string surfaceName_;
  double duration_ = 0;
  mc_trajectory::InterpolatedRotation oriSpline_;
  // Linear interpolation for gains
  SequenceInterpolator6d dimWeightInterpolator_;
  SequenceInterpolator6d stiffnessInterpolator_;
  SequenceInterpolator6d dampingInterpolator_;

  bool paused_ = false;

  double currTime_ = 0.;
  unsigned samples_ = 20;
  bool inSolver_ = false;
};
} // namespace mc_tasks

#include <mc_tasks/SplineTrajectoryTask.hpp>
