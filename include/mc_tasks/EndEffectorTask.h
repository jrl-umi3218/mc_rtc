#ifndef _H_MCTASKSEFTASK_H_
#define _H_MCTASKSEFTASK_H_

#include <mc_tasks/MetaTask.h>

#include <mc_rbdyn/robot.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

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
  EndEffectorTask(const std::string & bodyName, const mc_rbdyn::Robots &
                  robots, unsigned int robotIndex, double stiffness = 10.0,
                  double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the task objective to the current end-effector position
   */
  virtual void resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

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

  /*! \brief Returns the task error
   *
   * \returns Task error
   *
   */
  const Eigen::VectorXd& eval();

  /*! \brief Returns the task speed
   *
   * \returns Task speed
   *
   */
  const Eigen::VectorXd& speed();
public:
  const mc_rbdyn::Robots & robots;
  unsigned int robotIndex;

  std::shared_ptr<tasks::qp::PositionTask> positionTask;
  std::shared_ptr<tasks::qp::SetPointTask> positionTaskSp;
  std::shared_ptr<tasks::qp::OrientationTask> orientationTask;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp;

  std::string bodyName;
  sva::PTransformd curTransform;
  bool inSolver;
  Eigen::VectorXd err;
  Eigen::VectorXd spd;
};

}

#endif
