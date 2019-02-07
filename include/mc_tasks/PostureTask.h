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
  Eigen::VectorXd dimWeight() const override
  {
    return Eigen::VectorXd::Zero(0);
  }

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd eval() const override;

  Eigen::VectorXd speed() const override;

  /** Change posture objective */
  void posture(const std::vector<std::vector<double>> & p);

  /** Get current posture objective */
  std::vector<std::vector<double>> posture() const;

  /** Set joint gains for the posture task */
  void jointGains(const mc_solver::QPSolver & solver, const std::vector<tasks::qp::JointGains> & jgs);

  /** Set joint stiffness for the posture task */
  void jointStiffness(const mc_solver::QPSolver & solver, const std::vector<tasks::qp::JointStiffness> & jss);

  /** Set specific joint targets
   *
   * \param joints Map of joint's name to joint's configuration
   *
   */
  void target(const std::map<std::string, std::vector<double>> & joints);

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

  void addToGUI(mc_rtc::gui::StateBuilder &) override;

private:
  /** True if added to solver */
  bool inSolver_ = false;
  /** Robot handled by the task */
  const mc_rbdyn::Robots & robots_;
  unsigned int rIndex_;
  /** Actual task */
  tasks::qp::PostureTask pt_;
  /** Solver timestep */
  double dt_;
  /** Store the target posture */
  std::vector<std::vector<double>> posture_;
  /** Store mimic information */
  std::unordered_map<std::string, std::vector<int>> mimics_;
  /** Store the previous eval vector */
  Eigen::VectorXd eval_;
  /** Store the task speed */
  Eigen::VectorXd speed_;
};

} // namespace mc_tasks
