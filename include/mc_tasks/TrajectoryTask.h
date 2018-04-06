#pragma once

#include <mc_tasks/MetaTask.h>
#include <mc_trajectory/BSplineTrajectory.h>
#include <mc_rtc/GUIState.h>

#include <mc_rbdyn/Robots.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

/*! \brief This task moves a robot's surface from its current position to a target position following a b-spline.
 *
 * Waypoints can be provided or computed automatically
 *
 * \note Does NOT work at the moment
 *
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
   * \param timeStep Timestep of the controller
   *
   * \param stiffness Task stiffness (position and orientation)
   *
   * \param posWeight Task weight (position)
   *
   * \param oriWeight Task weight (orientation)
   *
   * \param waypoints Waypoints provided "by-hand". Each (3-rows) column of the
   * matrix will be used as a waypoint
   *
   * \param nrWP If this parameter is > 0 then nrWP waypoints are automatically
   * computed to smooth the trajectory. If waypoints were provided and nrWP > 0
   * then the provided waypoints are ignored.
   *
   */
  TrajectoryTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                 const std::string& surfaceName, const sva::PTransformd & X_0_t,
                 double duration, double timeStep, double stiffness, double posWeight, double oriWeight,
                 const Eigen::MatrixXd & waypoints = Eigen::MatrixXd(3,0),
                 unsigned int nrWP = 0);

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
   * \brief Final task target (trajectory end-point).
   *
   * return The target pose
   */
  sva::PTransformd target() const;

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
                                  const std::vector<std::string> &) override;

  void selectUnactiveJoints(mc_solver::QPSolver &,
                                    const std::vector<std::string> &) override;

  void resetJointsSelector(mc_solver::QPSolver &) override;

  void dimWeight(const Eigen::VectorXd &) override;
  Eigen::VectorXd dimWeight() const override { return dimWeight_; }

  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;

private:
  void generateBS();

  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update() override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

public:
  const mc_rbdyn::Robots & robots;
  unsigned int rIndex;
  std::string surfaceName;
  sva::PTransformd X_0_t;
  sva::PTransformd X_0_start;
  Eigen::MatrixXd wp;
  double duration;
  double timeStep;
  double t;
  std::shared_ptr<tasks::qp::JointsSelector> selectorT = nullptr;
  std::shared_ptr<tasks::qp::TransformTask> transTask = nullptr;
  std::shared_ptr<tasks::qp::TrajectoryTask> transTrajTask = nullptr;
  std::shared_ptr<mc_trajectory::BSplineTrajectory> bspline = nullptr;
  bool inSolver = false;
  double stiffness_;
  Eigen::Vector6d dimWeight_;
private:
  /* Hide these virtual functions */
  void reset() override {}
};

}
