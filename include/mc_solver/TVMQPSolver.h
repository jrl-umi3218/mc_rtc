/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/QPSolver.h>

#include <mc_rtc/clock.h>

#include <tvm/ControlProblem.h>
#include <tvm/LinearizedControlProblem.h>
#include <tvm/scheme/WeightedLeastSquares.h>

namespace mc_solver
{

/** This implements the QPSolver interface for the TVM backend
 *
 * This solver can accepts tasks and constraints from mc_rtc
 *
 */
struct MC_SOLVER_DLLAPI TVMQPSolver final : public QPSolver
{
  TVMQPSolver(mc_rbdyn::RobotsPtr robots, double timeStep);

  TVMQPSolver(double timeStep);

  ~TVMQPSolver() final = default;

  void gui(std::shared_ptr<mc_rtc::gui::StateBuilder> gui) override;

  void setContacts(ControllerToken, const std::vector<mc_rbdyn::Contact> & contacts) final;

  const sva::ForceVecd desiredContactForce(const mc_rbdyn::Contact & id) const final;
  const sva::ForceVecd desiredContactForce2(const mc_rbdyn::Contact & id) const;

  double solveTime() final;

  double solveAndBuildTime() final;

  /** Access the internal problem */
  inline tvm::LinearizedControlProblem & problem() noexcept { return problem_; }

  /** Access the internal problem (const) */
  inline const tvm::LinearizedControlProblem & problem() const noexcept { return problem_; }

  /** Access the dynamics constraints map */
  inline const std::unordered_map<std::string, DynamicsConstraint *> & dynamics() const noexcept { return dynamics_; };

  /** Helper to get a \ref TVMQPSolver from a \ref QPSolver instance
   *
   * The caller should make sure the cast is valid by checking the QPSolver backend.
   *
   * In debug the program will abort otherwise, in release UB abounds
   */
  static inline TVMQPSolver & from_solver(QPSolver & solver) noexcept
  {
    assert(solver.backend() == QPSolver::Backend::TVM);
    return static_cast<TVMQPSolver &>(solver);
  }

  /** Helper to get a \ref TVMQPSolver from a \ref QPSolver instance
   *
   * The caller should make sure the cast is valid by checking the QPSolver backend.
   *
   * In debug the program will abort otherwise, in release UB abounds
   */
  static inline const TVMQPSolver & from_solver(const QPSolver & solver) noexcept
  {
    assert(solver.backend() == QPSolver::Backend::TVM);
    return static_cast<const TVMQPSolver &>(solver);
  }

  /** Save the problem graph to a dot file that can be visualized with graphviz or other related tools
   *
   * The generated graph is located in <tmp>/mc_rtc_tvm_graph_<date>.dot
   *
   * To generate the graph, use (graphviz needs to be installed):
   * \code
   *    dot -Tps /tmp/mc_rtc_tvm_graph-latest.dot -o /tmp/tvm_graph.ps
   * \endcode
   *
   * @return true on success
   **/
  bool saveGraphDotFile() const;

  /** Same as saveGraphDotFile but with a custom filename */
  bool saveGraphDotFile(const std::string & filename) const;

private:
  /** Control problem */
  tvm::LinearizedControlProblem problem_;
  /** Solver scheme */
  tvm::scheme::WeightedLeastSquares solver_;
  /** Contact data on the solver side */
  struct ContactData
  {
    /** Contact function in the solver */
    tvm::TaskWithRequirementsPtr contactConstraint_;
    /** Force variables on r1 side (if any) */
    tvm::VariableVector f1_;
    /** Constraints on f1 */
    std::vector<tvm::TaskWithRequirementsPtr> f1Constraints_;
    /** Target tasks on f1 */
    std::vector<tvm::TaskWithRequirementsPtr> f1Targets_;
    /** Force variables on r2 side (if any) */
    tvm::VariableVector f2_;
    /** Constraints on f2 */
    std::vector<tvm::TaskWithRequirementsPtr> f2Constraints_;
    /** Target tasks on f2 */
    std::vector<tvm::TaskWithRequirementsPtr> f2Targets_;
  };
  /** Related contact functions */
  std::vector<ContactData> contactsData_;

  /** Runtime of the latest run call */
  mc_rtc::duration_ms solve_dt_{0};

  /** Common part of control loop */
  bool runCommon();
  /** Run without feedback (open-loop) */
  bool runOpenLoop();
  /** Run with encoders' feedback */
  bool runJointsFeedback(bool wVelocity);

  /**
   * WARNING EXPERIMENTAL
   *
   * Runs the QP on an estimated robot state.
   *
   * Uses the real robot state (mbc.q and mbc.alpha) from realRobots() instances.
   * It is the users responsibility to ensure that the real robot instance is properly estimated
   * and filled. Typically, this will be done through the Observers pipeline.
   * For example, the following pipeline provides a suitable state:
   *
   * \code{.yaml}
   * RunObservers: [Encoder, KinematicInertial]
   * UpdateObservers: [Encoder, KinematicInertial]
   * \endcode
   *
   * @param integrateControlState If true, integration is performed over the control state, otherwise over the observed
   * state
   *
   * @return True if successful, false otherwise
   */
  bool runClosedLoop(bool integrateControlState);

  /** Update a robot from the optimization result */
  void updateRobot(mc_rbdyn::Robot & robot);

  /** Feedback data */
  std::vector<std::vector<double>> prev_encoders_{};
  std::vector<std::vector<double>> encoders_alpha_{};
  std::vector<std::vector<std::vector<double>>> control_q_{};
  std::vector<std::vector<std::vector<double>>> control_alpha_{};

  /** Dynamics constraint currently active for robots in the solver */
  std::unordered_map<std::string, DynamicsConstraint *> dynamics_;

  bool run_impl(FeedbackType fType = FeedbackType::None) final;

  void addDynamicsConstraint(mc_solver::DynamicsConstraint * dynamics) final;

  void removeDynamicsConstraint(mc_solver::ConstraintSet * maybe_dynamics) final;
  void removeDynamicsConstraint(mc_solver::DynamicsConstraint * dyn);

  size_t getContactIdx(const mc_rbdyn::Contact & contact);
  void addContact(const mc_rbdyn::Contact & contact);
  using ContactIterator = std::vector<std::shared_ptr<mc_rbdyn::Contact>>::iterator;
  ContactIterator removeContact(size_t idx);

  /**
   * @brief Update or create and add an mc_tvm::ContactFunction (geometric constraint) to the problem,
   * and update solver contacts_ and contactsData_ vectors
   *
   * hasWork becomes true if friction or polytope changed, in this case dynamics function must be updated
   *
   * dofs changing do not influence the dynamics so does not trigger hasWork
   *
   * @param contact the mc_rbdyn::Contact to add or update
   * @return std::tuple of contact id / hasWork
   */
  std::tuple<size_t, bool> addVirtualContactImpl(const mc_rbdyn::Contact & contact);

  /**
   * @brief If the robot has a dynamic constraint, add the contact's influence to it.
   *
   * This creates force variables for each contact point and constraints on them (either friction cone
   * or feasiblePolytope) and adds them to the problem.
   *
   * The tvm dependency between the force variables and the DynamicFunction is done here with addContact
   *
   * @param robot Robot name
   * @param frame Contact frame
   * @param points Contact points in the frame's parent body's frame
   * @param forces Ref to where the forces tvm variables created by this contact will be stored
   * @param constraints Ref to where the constraints on these forces will be stored
   * @param targets Ref to where the target functions for these forces will be stored
   * @param contact mc_rbydn::Contact object for contact friction or feasiblePolytope
   * @param dir Contact direction
   */
  void addContactToDynamics(const std::string & robot,
                            const mc_rbdyn::RobotFrame & frame,
                            const std::vector<sva::PTransformd> & points,
                            tvm::VariableVector & forces,
                            std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                            std::vector<tvm::TaskWithRequirementsPtr> & targets,
                            mc_rbdyn::Contact & contact,
                            double dir);
};

/** Helper to get a \ref TVMQPSolver from a \ref QPSolver instance
 *
 * The caller should make sure the cast is valid by checking the QPSolver backend.
 *
 * In debug the program will abort otherwise, in release UB abounds
 */
inline TVMQPSolver & tvm_solver(QPSolver & solver) noexcept
{
  return TVMQPSolver::from_solver(solver);
}

/* const version */
inline const TVMQPSolver & tvm_solver(const QPSolver & solver) noexcept
{
  return TVMQPSolver::from_solver(solver);
}

} // namespace mc_solver
