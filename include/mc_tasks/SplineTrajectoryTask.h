#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>
#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_tasks
{

template<typename Derived>
struct SplineTrajectoryTask : public TrajectoryTaskGeneric<tasks::qp::TransformTask>
{
  using SplineTrajectoryBase = SplineTrajectoryTask<Derived>;
  using TrajectoryTask = TrajectoryTaskGeneric<tasks::qp::TransformTask>;

  /*! \brief Constructor
   *
   * \param robots Robots controlled by the task
   * \param robotIndex Which robot is controlled
   * \param surface Surface controlled by the task (should belong to the controlled robot)
   * \param duration_ Length of the movement
   * \param stiffness Task stiffness (position and orientation)
   * \param posW Task weight (position)
   * \param oriW Task weight (orientation)
   * \param oriWp Optional orientation waypoints
   */
  SplineTrajectoryTask(const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       const std::string & surfaceName_,
                       double duration_,
                       double stiffness,
                       double posW,
                       double oriW,
                       const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

  void oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);

  /*! \brief Weight for controlling position/orientation importance
   *
   * \param posWeight Task weight (position)
   */
  void posWeight(double posWeight);

  /*! \brief Weight for controlling position/orientation importance
   *
   * \return posWeight Task weight (position)
   */
  double posWeight() const;

  /*! \brief Weight for controlling position/orientation importance
   *
   * \param oriWeight Task weight (orientation)
   */
  void oriWeight(double oriWeight);

  /*! \brief Weight for controlling position/orientation importance
   * \return  oriWeight Task weight (orientation)
   */
  double oriWeight() const;

  /*! \brief Whether the trajectory has finished
   *
   * \returns True if the trajectory has finished
   */
  bool timeElapsed();

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

  // FIXME
  void target(const sva::PTransformd & target);
  const sva::PTransformd target() const;

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

  /*! \brief Get the control points of the trajectory's b-spline
   *
   * \returns The list of control points in the spline
   *
   */
  std::vector<Eigen::Vector3d> controlPoints();

  /**
   * \brief Number of points to sample on the spline for the gui display
   * \param s number of samples
   */
  void displaySamples(unsigned s);
  /**
   * \brief Number of samples for displaying the spline
   * \return number of samples
   */
  unsigned displaySamples() const;

protected:
  /**
   * \brief Add task controls to the GUI.
   * Interactive controls for the trajectory waypoints and end-endpoints
   * automatically updates the curve
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  /*! \brief Add the task to a solver
   * \param solver Solver where to add the task
   */
  void addToSolver(mc_solver::QPSolver & solver) override;
  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;

  /** \brief Update trajectory target
   */
  void update() override;

protected:
  mc_trajectory::InterpolatedRotation oriSpline_;
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp_;

protected:
  unsigned int rIndex_;
  std::string surfaceName_;
  sva::PTransformd initialPose_;
  sva::PTransformd finalTarget_;

  double duration_;
  double currTime_ = 0.;
  double timeStep_ = 0;
  unsigned samples_ = 20;
  bool inSolver_ = false;
};
} // namespace mc_tasks

#include <mc_tasks/SplineTrajectoryTask.hpp>
