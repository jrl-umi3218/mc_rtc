/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robots.h>
#include <mc_rtc/logging.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/api.h>

#include <Tasks/QPTasks.h>

namespace mc_tasks
{

/*! \brief Generic wrapper for a tasks::qp::TrajectoryTask
 *
 * This task is meant to be derived to build actual tasks
 *
 */
template<typename T>
struct TrajectoryTaskGeneric : public MetaTask
{
  using TrajectoryBase = TrajectoryTaskGeneric<T>;

  /*! \brief Constructor (auto damping)
   *
   * This is a simple constructor alternative. Damping is set to
   * 2*sqrt(stiffness). This is the most appropriate constructor to use
   * TrajectoryTask as a SetPointTask
   *
   * \param robots Robots used in the task
   *
   * \param robotIndex Index of the robot controlled by the task
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   */
  TrajectoryTaskGeneric(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight);

  virtual ~TrajectoryTaskGeneric();

  /*! \brief Reset task target velocity and acceleration to zero
   *
   */
  virtual void reset() override;

  /*! \brief Set the trajectory reference velocity
   *
   * \param vel New reference velocity
   *
   */
  void refVel(const Eigen::VectorXd & vel);

  /*! \brief Get the trajectory reference velocity
   *
   */
  const Eigen::VectorXd & refVel() const;

  /*! \brief Set the trajectory reference acceleration
   *
   * \param accel New reference acceleration
   *
   */
  void refAccel(const Eigen::VectorXd & accel);

  /*! \brief Get the trajectory reference acceleration
   *
   */
  const Eigen::VectorXd & refAccel() const;

  /*! \brief Set the task stiffness/damping
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Task stiffness
   *
   */
  void stiffness(double stiffness);

  /*! \brief Set dimensional stiffness
   *
   * The caller should be sure that the dimension of the vector fits the task dimension.
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Dimensional stiffness
   *
   */
  void stiffness(const Eigen::VectorXd & stiffness);

  /*! \brief Set the task damping, leaving its stiffness unchanged
   *
   * \param damping Task stiffness
   *
   */
  void damping(double damping);

  /*! \brief Set dimensional damping
   *
   * The caller should be sure that the dimension of the vector fits the task dimension.
   *
   * \param damping Dimensional damping
   *
   */
  void damping(const Eigen::VectorXd & damping);

  /*! \brief Set both stiffness and damping
   *
   * \param stiffness Task stiffness
   *
   * \param damping Task damping
   *
   */
  void setGains(double stiffness, double damping);

  /*! \brief Set dimensional stiffness and damping
   *
   * The caller should be sure that the dimensions of the vectors fit the task dimension.
   *
   * \param stiffness Dimensional stiffness
   *
   * \param damping Dimensional damping
   *
   */
  void setGains(const Eigen::VectorXd & stiffness, const Eigen::VectorXd & damping);

  /*! \brief Get the current task stiffness */
  double stiffness() const;

  /*! \brief Get the current task damping */
  double damping() const;

  /*! \brief Get the current task dimensional stiffness */
  const Eigen::VectorXd & dimStiffness() const;

  /*! \brief Get the current task dimensional damping */
  const Eigen::VectorXd & dimDamping() const;

  /*! \brief Set the task weight
   *
   * \param w Task weight
   *
   */
  void weight(double w);

  /*! \brief Returns the task weight */
  double weight() const;

  virtual void dimWeight(const Eigen::VectorXd & dimW) override;

  virtual Eigen::VectorXd dimWeight() const override;

  /** \brief Create an active joints selector
   *
   * \warning This function should only be called if the task hasn't yet been
   * added to the solver. If the tasks is already in the solver it does nothing,
   * and warns that it had no effect. Call void selectActiveJoints(mc_solver::QPSolver &, const std::vector<std::string>
   * &, const std::map<std::string, std::vector<std::array<int, 2>>> &) instead.
   *
   * @param activeJointsName Name of the joints activated for this task
   * \param activeDofs Allow to select only part of the dofs of a joint
   * \param checkJoints When true, checks whether the joints exist in the robot
   * \throws If checkJoints is true and a joint name in activeJointsName is not
   * part of the robot
   */
  virtual void selectActiveJoints(const std::vector<std::string> & activeJointsName,
                                  const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {},
                                  bool checkJoints = true);

  /** \brief Create an active joints selector
   *
   * \note Calling this function is a bit expensive. If the task is already in
   * the solver, it will be removed first, then recreated with the joint
   * selector and added to the solver again. If possible, consider calling void selectActiveJoints(const
   * std::vector<std::string> &, const std::map<std::string, std::vector<std::array<int, 2>>> &) before adding the task
   * to the solver.
   *
   * @param activeJointsName Name of the joints activated for this task
   * \param activeDofs Allow to select only part of the dofs of a joint
   * \throws If a joint name in activeJointsName is not part of the robot
   */
  virtual void selectActiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & activeJointsName,
      const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  /** \brief Create an unactive joints selector
   *
   * \warning This function should only be called if the task hasn't yet been
   * added to the solver. If the tasks is already in the solver it does nothing,
   * and warns that it had no effect. Call void selectUnactiveJoints(mc_solver::QPSolver &, const
   * std::vector<std::string> &, const std::map<std::string, std::vector<std::array<int, 2>>> &) instead.
   *
   * @param unactiveJointsName Name of the joints not activated for this task
   * \param unactiveDofs Allow to select only part of the dofs of a joint
   * \param checkJoints When true, checks whether the joints exist in the robot
   * \throws If checkJoints is true and a joint name in unactiveJointsName is not
   * part of the robot
   */
  virtual void selectUnactiveJoints(const std::vector<std::string> & unactiveJointsName,
                                    const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {},
                                    bool checkJoints = true);

  /** \brief Create an unactive joints selector
   *
   * \note Calling this function is a bit expensive. If the task is already in
   * the solver, it will be removed first, then recreated with the joint
   * selector and added to the solver again. If possible, consider calling void selectUnactiveJoints(const
   * std::vector<std::string> &, const std::map<std::string, std::vector<std::array<int, 2>>> &) before adding the task
   * to the solver.
   *
   * @param unactiveJointsName Name of the joints not activated for this task
   * \param unactiveDofs Allow to select only part of the dofs of a joint
   * \throws If a joint name in unactiveJointsName is not part of the robot
   */
  virtual void selectUnactiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & unactiveJointsName,
      const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  virtual void resetJointsSelector();

  virtual void resetJointsSelector(mc_solver::QPSolver & solver) override;

  virtual Eigen::VectorXd eval() const override;

  virtual Eigen::VectorXd speed() const override;

  const Eigen::VectorXd & normalAcc() const;

  const Eigen::MatrixXd & jac() const;

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  /*! This function should be called to finalize the task creation, it will
   * create the actual tasks objects */
  template<typename... Args>
  void finalize(Args &&... args);

  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  void addToLogger(mc_rtc::Logger & logger) override;

  void removeFromLogger(mc_rtc::Logger & logger) override;

  std::function<bool(const mc_tasks::MetaTask & task, std::string &)> buildCompletionCriteria(
      double dt,
      const mc_rtc::Configuration & config) const override;

protected:
  const mc_rbdyn::Robots & robots;
  unsigned int rIndex;
  std::shared_ptr<T> errorT = nullptr;
  Eigen::VectorXd refVel_;
  Eigen::VectorXd refAccel_;
  bool inSolver_ = false;
  std::shared_ptr<tasks::qp::TrajectoryTask> trajectoryT_ = nullptr;

protected:
  virtual void addToSolver(mc_solver::QPSolver & solver) override;

private:
  Eigen::VectorXd stiffness_;
  Eigen::VectorXd damping_;
  double weight_;
  std::shared_ptr<tasks::qp::JointsSelector> selectorT_ = nullptr;

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver &) override;
};

} // namespace mc_tasks

#include <mc_tasks/TrajectoryTaskGeneric.hpp>
