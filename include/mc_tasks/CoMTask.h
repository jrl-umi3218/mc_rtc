#ifndef _H_MCTASKCOMTASK_H_
#define _H_MCTASKCOMTASK_H_

#include <mc_rbdyn/robot.h>
#include <mc_solver/QPSolver.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>
#include <mc_tasks/MetaTask.h>

namespace mc_tasks
{

/*! \brief Control a robot's CoM
 *
 * This task is a thin wrapper above the appropriate Tasks types
 *
 */
struct MC_TASKS_DLLAPI CoMTask : public MetaTask
{
public:
  /*! \brief Constructor
   *
   * \param robots Robots involved in the task
   *
   * \param robotIndex Select the robot which CoM should be controlled
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex,
          double stifness = 5.0, double weight = 100.);

  /*! \brief Reset the task
   *
   * Set the CoM target to the current configuration's CoM
   *
   * \param robots Robots involved in the task
   *
   * \param robotIndex Select the robot which CoM should be controlled
   *
   */
  void resetTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex);

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

  /*! \brief Change the CoM target by a given amount
   *
   * \param com Modification applied to the CoM
   *
   */
  void move_com(const Eigen::Vector3d & com);

  /*! \brief Set the CoM target to a given position
   *
   * \param com New CoM target
   *
   */
  void set_com(const Eigen::Vector3d & com);

  /*! \brief Return the current CoM target
   *
   * \returns The current CoM target
   */
  Eigen::Vector3d get_com();
public:
  const mc_rbdyn::Robots & robots;

  std::shared_ptr<tasks::qp::CoMTask> comTask;
  std::shared_ptr<tasks::qp::SetPointTask> comTaskSp;

  Eigen::Vector3d cur_com;
  bool in_solver;
};

}

#endif
