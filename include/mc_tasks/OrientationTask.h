#ifndef _H_MCTASKSORITASK_H_
#define _H_MCTASKSORITASK_H_

#include <mc_rbdyn/robot.h>
#include <mc_solver/QPSolver.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>
#include <mc_tasks/MetaTask.h>

namespace mc_tasks
{

/*! \brief Control the orientation of a body
 *
 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI OrientationTask : public MetaTask
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
  OrientationTask(const std::string & bodyName, const mc_rbdyn::Robots
                  & robots, unsigned int robotIndex, double stiffness =
                  2.0, double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task objective to the current body orientation
   */
  void resetTask();

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

  /*! \brief Set the body orientation target
   *
   * \param ori Body orientation in world frame
   *
   */
  void set_ef_ori(const Eigen::Matrix3d & ori);

  /*! \brief Get the current body orientation target
   *
   * \returns The body orientation target in world frame
   *
   */
  Eigen::Matrix3d get_ef_ori();
public:
  std::string bodyName;
  const mc_rbdyn::Robots & robots;
  unsigned int rIndex;
  unsigned int bIndex;

  bool inSolver;
  std::shared_ptr<tasks::qp::OrientationTask> orientationTask;
  std::shared_ptr<tasks::qp::SetPointTask> orientationTaskSp;
};

}

#endif
