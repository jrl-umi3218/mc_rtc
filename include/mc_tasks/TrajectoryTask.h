/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robots.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/api.h>
#include <mc_trajectory/BSplineConstrainedTrajectory.h>
#include <mc_trajectory/BSplineTrajectory.h>
#include <mc_trajectory/InterpolatedTrajectory.h>

#include <Tasks/QPTasks.h>

namespace mc_tasks
{

/*! \brief This task moves a robot's surface from its current position to a target position following a b-spline.
 *
 * Waypoints can be provided or computed automatically
 */
struct MC_TASKS_DLLAPI TrajectoryTask : public MetaTask
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

  /*! \brief Set the task stiffness/damping
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Task stiffness
   *
   */
  void stiffness(double stiffness);

  /*! \brief Get the current task stiffness */
  double stiffness() const;

  /*! \brief Set the task damping, leaving its stiffness unchanged
   *
   * \param damping Task stiffness
   *
   */
  void damping(double damping);

  /*! \brief Get the current task damping */
  double damping() const;

  /*! \brief Set both stiffness and damping
   *
   * \param stiffness Task stiffness
   *
   * \param damping Task damping
   *
   */
  void setGains(double stiffness, double damping);

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

  /*! \brief Returns the current task speed
   *
   * \returns The current task speed
   *
   */
  Eigen::VectorXd speed() const override;

  void target(const sva::PTransformd & target);
  const sva::PTransformd & target() const;

  void refVel(const Eigen::VectorXd & vel);
  const Eigen::VectorXd & refVel() const;

  void refAcc(const Eigen::VectorXd & acc);
  const Eigen::VectorXd & refAcc() const;

  void refTarget(const sva::PTransformd & target);
  const sva::PTransformd & refTarget() const;

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

  void selectActiveJoints(mc_solver::QPSolver &,
                          const std::vector<std::string> &,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  void selectUnactiveJoints(mc_solver::QPSolver &,
                            const std::vector<std::string> &,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver &) override;

  void dimWeight(const Eigen::VectorXd &) override;
  Eigen::VectorXd dimWeight() const override;

  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;

protected:
  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update() override;

  /**
   * \brief Add task controls to the GUI.
   * Interactive controls for the trajectory waypoints and end-endpoints
   * automatically call generateBS() to update the curve
   *
   * \param gui
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui) override;

  void addToSolver(mc_solver::QPSolver & solver) override;

public:
  const mc_rbdyn::Robots & robots;
  unsigned int rIndex;
  std::string surfaceName;
  sva::PTransformd X_0_t;
  sva::PTransformd X_0_start;

  double stiffness_;
  double damping_;

  double duration;
  double t = 0.;
  double timeStep = 0;
  unsigned samples_ = 20;
  std::shared_ptr<tasks::qp::JointsSelector> selectorT = nullptr;
  std::shared_ptr<tasks::qp::TransformTask> transTask = nullptr;
  std::shared_ptr<tasks::qp::TrajectoryTask> transTrajTask = nullptr;
  bool inSolver = false;

private:
  /* Hide these virtual functions */
  void reset() override {}
};

} // namespace mc_tasks
