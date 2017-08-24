#ifndef _H_STABILITYTASK_H_
#define _H_STABILITYTASK_H_

#include <mc_tasks/MetaTask.h>
#include <mc_tasks/SmoothTask.h>

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/StanceConfig.h>

#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

/*! \brief This task is a combination of a tasks::qp::PostureTask and a
 * mc_tasks::CoMTask that is used as a stabilizing task for the Seq
 * controller
 *
 * It is not advised to use this task outside of the Seq controller. In
 * fact, it is voluntarily made hard to do so.
 *
 */
struct MC_TASKS_DLLAPI StabilityTask : public MetaTask
{
public:
  /*! Constructor
   *
   * \param robots Robots controlled by this task
   *
   */
  StabilityTask(mc_rbdyn::Robots & robots);

  /*! Update the task target
   *
   * \param env Unused
   *
   * \param stance Target stance
   *
   * \param config Corresponding stance configuration
   *
   * \param comSmoothPercent Smooth task interpolation percentage
   *
   */
  void target(const mc_rbdyn::Robot & env, const mc_rbdyn::Stance & stance,
              const mc_rbdyn::StanceConfig & config, double comSmoothPercent = 1);

  virtual void reset() override;

  /*! \brief Set a high stiffness for selected joints
   *
   * That is a stiffness 10 times as high as the posture task
   *
   * \param stiffJoints Joints whose stiffness should be modified
   *
   */
  void highStiffness(const std::vector<std::string> & stiffJoints);

  /*! \brief Set a normal stiffness for selected joints
   *
   * That is the same stiffness as the posture task
   *
   * \param stiffJoints Joints whose stiffness should be modified
   *
   */
  void normalStiffness(const std::vector<std::string> & stiffJoints);

  virtual Eigen::VectorXd eval() const override;

  virtual Eigen::VectorXd speed() const override;
public:
  mc_rbdyn::Robots & robots;
  mc_rbdyn::Robot & robot;

  /* CoMTask */
  double comStiff;
  double extraComStiff;
  Eigen::Vector3d comObj;
  std::shared_ptr<tasks::qp::CoMTask> comTask;
  std::shared_ptr<tasks::qp::SetPointTask> comTaskSp;
  SmoothTask<Eigen::Vector3d> comTaskSm;

  std::vector< std::vector<double> > qObj;
  std::shared_ptr<tasks::qp::PostureTask> postureTask;
private:
  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

  /* Hide these virtual functions */
  virtual void dimWeight(const Eigen::VectorXd & /*dimW*/) override {}

  virtual Eigen::VectorXd dimWeight() const override { return Eigen::VectorXd(); }

  virtual void selectActiveJoints(mc_solver::QPSolver &,
                                  const std::vector<std::string> &) override {}

  virtual void selectUnactiveJoints(mc_solver::QPSolver &,
                                    const std::vector<std::string> &) override {}

  virtual void resetJointsSelector(mc_solver::QPSolver &) override {}
};

}

#endif
