#pragma once

#include <mc_tasks/MetaTask.h>

#include <Tasks/QPTasks.h>

namespace mc_tasks
{

/** A posture task for a given robot */
struct MC_TASKS_DLLAPI PostureTask : public MetaTask
{
public:
  PostureTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double stiffness, double weight);

  void reset() override;

  /** Does not make much sense for PostureTask, prefer selectActiveJoints
   * or selectUnactiveJoints */
  void dimWeight(const Eigen::VectorXd &) override {}

  /** Does not make much sense for PostureTask, prefer selectActiveJoints
   * or selectUnactiveJoints */
  Eigen::VectorXd dimWeight() const override { return Eigen::VectorXd::Zero(0); }

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName) override;

  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd eval() const override;

  Eigen::VectorXd speed() const override;

  /** Change posture objective */
  void posture(const std::vector<std::vector<double>> & p);

  /** Get current posture objective */
  std::vector<std::vector<double>> posture() const;

  /** Set task's stiffness */
  void stiffness(double s);

  /** Get task's stiffness */
  double stiffness() const;

  /** Set task's weight */
  void weight(double w);

  /** Get task's weight */
  double weight() const;

  /** True if the task is in the solver */
  bool inSolver() const;
protected:
  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update() override;
private:
  /** True if added to solver */
  bool inSolver_ = false;
  /** Robot handled by the task */
  const mc_rbdyn::Robot & robot_;
  /** Actual task */
  tasks::qp::PostureTask pt_;
  /** Solver timestep */
  double dt_;
  /** Store the previous eval vector */
  Eigen::VectorXd eval_;
  /** Store the task speed */
  Eigen::VectorXd speed_;
};

}
