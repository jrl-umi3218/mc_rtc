/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/MetaTask.h>

#include <Tasks/QPTasks.h>

namespace mc_tasks
{

/** A posture task for a given robot */
struct MC_TASKS_DLLAPI PostureTask : public MetaTask
{
public:
  PostureTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double stiffness = 1, double weight = 10);

  void reset() override;

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

  /*! \brief Set the task dimensional weight
   *
   * For simple cases (using 0/1 as weights) prefer \ref selectActiveJoints or \ref selectUnactiveJoints which are
   * simpler to use
   */
  inline void dimWeight(const Eigen::VectorXd & dimW) override
  {
    pt_.dimWeight(dimW);
  }

  Eigen::VectorXd dimWeight() const override
  {
    return pt_.dimWeight();
  }

  /*! \brief Select active joints for this task
   *
   * Manipulate \ref dimWeight() to achieve its effect
   */
  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  /*! \brief Select inactive joints for this task
   *
   * Manipulate \ref dimWeight() to achieve its effect
   */
  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  /*! \brief Reset the joint selector effect
   *
   * Reset dimWeight to ones
   */
  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd eval() const override;

  Eigen::VectorXd speed() const override;

  /** Change posture objective */
  void posture(const std::vector<std::vector<double>> & p);

  /** Change reference velocity
   *
   * \p refVel Should be of size nrDof
   */
  inline void refVel(const Eigen::VectorXd & refVel) noexcept
  {
    assert(refVel.size() == robots_.robot(rIndex_).mb().nrDof());
    pt_.refVel(refVel);
  }

  /** Access the reference velocity */
  inline const Eigen::VectorXd & refVel() const noexcept
  {
    return pt_.refVel();
  }

  /** Change reference acceleration
   *
   * \p refAccel Should be of size nrDof
   */
  inline void refAccel(const Eigen::VectorXd & refAccel) noexcept
  {
    assert(refAccel.size() == robots_.robot(rIndex_).mb().nrDof());
    pt_.refAccel(refAccel);
  }

  /** Access the reference acceleration */
  inline const Eigen::VectorXd & refAccel() const noexcept
  {
    return pt_.refAccel();
  }

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

  /** Set the task stiffness/damping
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Task stiffness
   *
   */
  void stiffness(double s);

  /** Get the current task's stiffness */
  double stiffness() const;

  /** Set the task damping, leaving its stiffness unchanged
   *
   * \param damping Task stiffness
   *
   */
  void damping(double d);

  /** Get the task's current damping */
  double damping() const;

  /** Set both stiffness and damping
   *
   * \param stiffness Task stiffness
   *
   * \param damping Task damping
   *
   */
  void setGains(double stiffness, double damping);

  /** Set task's weight */
  void weight(double w);

  /** Get task's weight */
  double weight() const;

  /** True if the task is in the solver */
  bool inSolver() const;

protected:
  void addToSolver(mc_solver::QPSolver & solver) override;

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver &) override;

  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  void addToLogger(mc_rtc::Logger & logger) override;

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

using PostureTaskPtr = std::shared_ptr<PostureTask>;

} // namespace mc_tasks
