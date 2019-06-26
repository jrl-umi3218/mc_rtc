/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robots.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/api.h>
#include <mc_trajectory/BSplineTrajectory.h>

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
   * \param X_0_t Target position of the controlled surface (world frame)
   *
   * \param duration Length of the movement
   *
   * \param stiffness Task stiffness (position and orientation)
   *
   * \param posW Task weight (position)
   *
   * \param oriW Task weight (orientation)
   *
   * \param waypoints Waypoints provided "by-hand". Each (3-rows) column of the
   * matrix will be used as a waypoint
   *
   * \param oriWpTime Time at which each rotation waypoint will be activated.
   * If left empty, rotation will be interpolated according to the starting and
   * end pose for the trajectory.
   * Should have the same size as oriWp
   *
   * \param oriWp Rotation matrix (world frame) for each rotation waypoint.
   * If left empty, rotation will be interpolated according to the starting and
   * end pose for the trajectory.
   * Should have the same size as oriWpTime
   *
   */
  TrajectoryTask(const mc_rbdyn::Robots & robots,
                 unsigned int robotIndex,
                 const std::string & surfaceName,
                 const sva::PTransformd & X_0_t,
                 double duration,
                 double stiffness,
                 double posW,
                 double oriW,
                 const Eigen::MatrixXd & waypoints,
                 const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

  /**
   * \brief Constructor for TrajectoryTask with automatic waypoint generation
   * (position), and optional waypoints in orientation
   *
   * \param nrWP nrWP waypoints are automatically computed to smooth the trajectory.
   */
  TrajectoryTask(const mc_rbdyn::Robots & robots,
                 unsigned int robotIndex,
                 const std::string & surfaceName,
                 const sva::PTransformd & X_0_t,
                 double duration,
                 double stiffness,
                 double posW,
                 double oriW,
                 unsigned int nrWP,
                 const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

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

  /**
   * \brief Sets the final trajectory target
   * Calls generateBS()
   *
   * \param target Target pose in world coordinates
   */
  void target(const sva::PTransformd & target);

  /**
   * \brief Final task target (trajectory end-point).
   *
   * return The target pose
   */
  const sva::PTransformd & target() const;

  /*! \brief Returns the current task speed
   *
   * \returns The current task speed
   *
   */
  Eigen::VectorXd speed() const override;

  /*! \brief Get the control points of the trajectory's b-spline
   *
   * \returns The list of control points in the spline
   *
   */
  std::vector<Eigen::Vector3d> controlPoints();

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

private:
  /**
   * \brief Actual task initialization constructor
   */
  void init(unsigned int robotIndex,
            const std::string & surfaceName,
            const sva::PTransformd & X_0_t,
            double duration,
            double stiffness,
            double posW,
            double oriW,
            const Eigen::MatrixXd & waypoints,
            const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);

  /**
   * \brief Generates the BSpline trajectory
   * The trajectory is defined as follows:
   * - Starting pose is (by default) the intial surface pose
   * - Control points are user-specified
   * - End point is set with target()
   *
   * Each update() call will provide a new target along this trajectory
   * tracked by the appropriate tasks. This target is interpolated along a
   * b-spline trajectory for the translational part. Rotation is interpolated
   * independently with optional waypoints.
   */
  void generateBS();

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update() override;

protected:
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
  const mc_rbdyn::Robots & robots;
  unsigned int rIndex;
  std::string surfaceName;
  sva::PTransformd X_0_t;
  sva::PTransformd X_0_start;
  Eigen::MatrixXd wp;

  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp_;
  sva::PTransformd X_0_oriStart;
  sva::PTransformd X_0_oriTarget;
  unsigned int oriTargetWpIndex;
  double oriStartTime;
  double oriDuration;
  double stiffness_;
  double damping_;

  double duration;
  double t = 0.;
  double timeStep = 0;
  std::shared_ptr<tasks::qp::JointsSelector> selectorT = nullptr;
  std::shared_ptr<tasks::qp::TransformTask> transTask = nullptr;
  std::shared_ptr<tasks::qp::TrajectoryTask> transTrajTask = nullptr;
  std::shared_ptr<mc_trajectory::BSplineTrajectory> bspline = nullptr;
  bool inSolver = false;

private:
  /* Hide these virtual functions */
  void reset() override {}
};

} // namespace mc_tasks
