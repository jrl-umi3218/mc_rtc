/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/MetaTask.h>

#include <mc_tasks/TVMTrajectoryTaskGeneric.h>

#include <mc_rbdyn/Robots.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/void_ptr.h>

#include <Tasks/QPTasks.h>

namespace mc_tasks
{

/*! \brief Generic wrapper for a trajectory dynamic over an error function
 *
 * This task is meant to be derived to build actual tasks
 *
 */
struct MC_TASKS_DLLAPI TrajectoryTaskGeneric : public MetaTask
{
  /** For backward compatibility */
  using TrajectoryBase = TrajectoryTaskGeneric;

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

  /*! \brief Constructor (auto damping)
   *
   * This is a simple constructor alternative. Damping is set to
   * 2*sqrt(stiffness). This is the most appropriate constructor to use
   * TrajectoryTask as a SetPointTask
   *
   * It is a useful shortcut when the task is controlling a frame
   *
   * \param frame Frame used in the task
   *
   * \param stiffness Stiffness of the task
   *
   * \param weight Weight of the task
   *
   */
  TrajectoryTaskGeneric(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight);

  virtual ~TrajectoryTaskGeneric() = default;

  /*! \brief Reset task target velocity and acceleration to zero
   *
   */
  void reset() override;

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

  void dimWeight(const Eigen::VectorXd & dimW) override;

  Eigen::VectorXd dimWeight() const override;

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
  void selectActiveJoints(const std::vector<std::string> & activeJointsName,
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
  void selectActiveJoints(mc_solver::QPSolver & solver,
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
  void selectUnactiveJoints(const std::vector<std::string> & unactiveJointsName,
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
  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  virtual void resetJointsSelector();

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd eval() const override;

  Eigen::VectorXd speed() const override;

  const Eigen::VectorXd & normalAcc() const;

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  /*! This function should be called to finalize the task creation, it will
   * create the actual tasks objects */
  /** This function must be called by the derived class to finalize the creation of the task
   *
   * \tparam ErrorT Actual type of \ref errorT
   *
   * \param args Arguments passed to the constructor of \ref errorT
   */
  template<Backend backend, typename ErrorT, typename... Args>
  inline void finalize(Args &&... args)
  {
    assert(backend == backend_);
    errorT = mc_rtc::make_void_ptr<ErrorT>(std::forward<Args>(args)...);
    if constexpr(backend == Backend::Tasks)
    {
      trajectoryT_ = mc_rtc::make_void_ptr<tasks::qp::TrajectoryTask>(
          robots.mbs(), rIndex, static_cast<ErrorT *>(errorT.get()), stiffness_(0), damping_(0), weight_);
      auto & trajectory = *static_cast<tasks::qp::TrajectoryTask *>(trajectoryT_.get());
      stiffness_ = trajectory.stiffness();
      damping_ = trajectory.damping();
      if(refVel_.size() != trajectory.refVel().size()) { refVel_ = trajectory.refVel(); }
      if(refAccel_.size() != trajectory.refAccel().size()) { refAccel_ = trajectory.refAccel(); }
    }
    else if constexpr(backend == Backend::TVM)
    {
      auto error = static_cast<ErrorT *>(errorT.get());
      trajectoryT_ = mc_rtc::make_void_ptr<details::TVMTrajectoryTaskGenericPtr>(
          std::make_shared<details::TVMTrajectoryTaskGeneric>());
      static_cast<details::TVMTrajectoryTaskGenericPtr *>(trajectoryT_.get())->get()->init<ErrorT>(error);
      int size = error->size();
      stiffness_ = Eigen::VectorXd::Constant(size, 1, stiffness_(0));
      damping_ = Eigen::VectorXd::Constant(size, 1, damping_(0));
      if constexpr(details::has_refVel_v<ErrorT>) { refVel_ = error->refVel(); }
      if constexpr(details::has_refAccel_v<ErrorT>) { refAccel_ = error->refAccel(); }
    }
    else
    {
      mc_rtc::log::error_and_throw("[{} task] Not implemented for backend: {}", backend_);
    }
  }

  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  void addToLogger(mc_rtc::Logger & logger) override;

  std::function<bool(const mc_tasks::MetaTask & task, std::string &)> buildCompletionCriteria(
      double dt,
      const mc_rtc::Configuration & config) const override;

  const mc_rbdyn::Robots & robots;
  unsigned int rIndex;
  /** Pointer to the error function
   *
   * The actual type depends on the implementation
   */
  mc_rtc::void_ptr errorT{nullptr, nullptr};
  Eigen::VectorXd refVel_;
  Eigen::VectorXd refAccel_;
  bool inSolver_ = false;
  /** Pointer to the trajectory dynamic
   *
   * In Tasks backend:
   * - tasks::qp::TrajectoryTask
   *
   * In TVM backend:
   * - details::TVMTrajectoryTaskGeneric
   */
  mc_rtc::void_ptr trajectoryT_{nullptr, nullptr};

  void addToSolver(mc_solver::QPSolver & solver) override;

  Eigen::VectorXd stiffness_;
  Eigen::VectorXd damping_;
  double weight_;
  /** Pointer to a dynamic wrapper that select specific joints
   *
   * In Tasks backend:
   * - tasks::qp::JointSelector
   *
   * In TVM backend:
   * mc_tvm::JointsSelectorFunction
   */
  mc_rtc::void_ptr selectorT_{nullptr, nullptr};

  void removeFromSolver(mc_solver::QPSolver & solver) override;

  void update(mc_solver::QPSolver &) override;
};

} // namespace mc_tasks
