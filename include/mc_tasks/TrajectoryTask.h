/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robots.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/TrajectoryTaskGeneric.h>
#include <mc_tasks/api.h>

#include <Tasks/QPTasks.h>

namespace mc_tasks
{

/*! \brief This task moves a robot's surface from its current position to a target position following a b-spline.
 *
 * Waypoints can be provided or computed automatically
 */
struct MC_TASKS_DLLAPI TrajectoryTask : public TrajectoryTaskGeneric<tasks::qp::TransformTask>
{
public:
  /*! \brief Constructor
   *
   * \param robots Robots controlled by the task
   *
   * \param robotIndex Which robot is controlled
   *
   * \param surface Surface controlled by the task, should belong to the
   * controlled robot
   *
   * \param duration Length of the movement
   *
   * \param stiffness Task stiffness (position and orientation)
   *
   * \param posW Task weight (position)
   *
   * \param oriW Task weight (orientation)
   *
   */
  TrajectoryTask(const mc_rbdyn::Robots & robots,
                 unsigned int robotIndex,
                 const std::string & surfaceName,
                 double duration,
                 double stiffness,
                 double posW,
                 double oriW);

  /*! \brief Weight for controlling position/orienation importance
   *
   * \param posWeight Task weight (position)
   */
  void posWeight(double posWeight);

  /*! \brief Weight for controlling position/orienation importance
   *
   * \return  posWeight Task weight (position)
   */
  double posWeight() const;

  /*! \brief Weight for controlling position/orienation importance
   *
   * \param oriWeight Task weight (orientation)
   */
  void oriWeight(double oriWeight);

  /*! \brief Weight for controlling position/orienation importance
   * \return  oriWeight Task weight (orientation)
   */
  double oriWeight() const;

  /*! \brief Whether the trajectory has been fully fed to the task or not
   *
   * \returns True if the trajectory has been fully fed.
   *
   */
  bool timeElapsed();

  /*! \brief Returns the transformError between current robot surface pose and
   * its final target
   *
   * \returns The error w.r.t the final target
   *
   */
  Eigen::VectorXd eval() const override;

  /**
   * \brief Returns the trajectory tracking error: transformError between the current robot surface pose
   * and its next desired pose along the trajectory error
   *
   * \return The trajectory tracking error
   */
  virtual Eigen::VectorXd evalTracking() const;

  void target(const sva::PTransformd & target);
  const sva::PTransformd & target() const;

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

  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;

protected:
  void update() override;

  /**
   * \brief Add task controls to the GUI.
   * Interactive controls for the trajectory waypoints and end-endpoints
   * automatically call generateBS() to update the curve
   *
   * \param gui
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToSolver(mc_solver::QPSolver & solver) override;

public:
  unsigned int rIndex;
  std::string surfaceName;
  sva::PTransformd X_0_t;
  sva::PTransformd X_0_start;

  double duration;
  double t = 0.;
  double timeStep = 0;
  unsigned samples_ = 20;
  bool inSolver = false;

private:
  /* Hide these virtual functions */
  void reset() override {}
};

} // namespace mc_tasks
