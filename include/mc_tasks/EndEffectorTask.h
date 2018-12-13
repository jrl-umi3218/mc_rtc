#pragma once

#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/PositionTask.h>

namespace mc_tasks
{

/*! \brief Controls an end-effector
 *
 * This task is a thin wrapper around the appropriate tasks in Tasks.
 * The task objective is given in the world frame. For relative control
 * see mc_tasks::RelativeEndEffectorTask
 */
struct MC_TASKS_DLLAPI EndEffectorTask : public MetaTask
{
public:
  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  EndEffectorTask(const std::string & bodyName,
                  const mc_rbdyn::Robots & robots,
                  unsigned int robotIndex,
                  double stiffness = 2.0,
                  double weight = 1000.0);

  /*! \brief Constructor with bodyPoint
   *
   * \param bodyPoint Point to be controlled in body coordinates
   *
   * @see EndEffectorTask
   *
   */
  EndEffectorTask(const std::string & bodyName,
                  const Eigen::Vector3d & bodyPoint,
                  const mc_rbdyn::Robots & robots,
                  unsigned int robotIndex,
                  double stiffness = 2.0,
                  double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the task objective to the current end-effector position
   */
  virtual void reset() override;

  /*! \brief Increment the target position
   *
   * \param dtr Change in target position
   *
   */
  virtual void add_ef_pose(const sva::PTransformd & dtr);

  /*! \brief Change the target position
   *
   * \param tf New target position
   *
   */
  virtual void set_ef_pose(const sva::PTransformd & tf);

  /*! \brief Returns the current target positions
   *
   * \returns Current target position
   *
   */
  virtual sva::PTransformd get_ef_pose();

  virtual void dimWeight(const Eigen::VectorXd & dimW) override;

  virtual Eigen::VectorXd dimWeight() const override;

  virtual void selectActiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & activeJointsName,
      const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  virtual void selectUnactiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & unactiveJointsName,
      const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  virtual void resetJointsSelector(mc_solver::QPSolver & solver) override;

  virtual Eigen::VectorXd eval() const override;

  virtual Eigen::VectorXd speed() const override;

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

public:
  const mc_rbdyn::Robots & robots;
  unsigned int robotIndex;
  unsigned int bodyIndex;

  std::shared_ptr<mc_tasks::PositionTask> positionTask;
  std::shared_ptr<mc_tasks::OrientationTask> orientationTask;

  std::string bodyName;
  sva::PTransformd curTransform;

protected:
  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

  virtual void addToLogger(mc_rtc::Logger & logger) override;
  virtual void removeFromLogger(mc_rtc::Logger & logger) override;

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
};

} // namespace mc_tasks
