/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>
#include <mc_control/fsm/Executor.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>

#include <functional>

namespace mc_control
{

namespace fsm
{

struct Controller;

/** \class Contact
 *
 * A lightweight variant of mc_rbdyn::Contact meant to simplify contact
 * manipulation in the FSM
 *
 */
struct MC_CONTROL_FSM_DLLAPI Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Contact(const std::string & r1,
          const std::string & r2,
          const std::string & r1Surface,
          const std::string & r2Surface,
          double friction = mc_rbdyn::Contact::defaultFriction,
          const Eigen::Vector6d & dof = Eigen::Vector6d::Ones())
  : r1(r1), r2(r2), r1Surface(r1Surface), r2Surface(r2Surface), friction(friction), dof(dof)
  {
  }

  std::string r1;
  std::string r2;
  std::string r1Surface;
  std::string r2Surface;
  mutable double friction;
  mutable Eigen::Vector6d dof;

  bool operator<(const Contact & rhs) const
  {
    return r1 < rhs.r1 || (r1 == rhs.r1 && r1Surface < rhs.r1Surface)
           || (r1 == rhs.r1 && r1Surface == rhs.r1Surface && r2 < rhs.r2)
           || (r1 == rhs.r1 && r1Surface == rhs.r1Surface && r2 == rhs.r2 && r2Surface < rhs.r2Surface);
  }

  bool operator==(const Contact & rhs) const
  {
    return r1 == rhs.r1 && r2 == rhs.r2 && r1Surface == rhs.r1Surface && r2Surface == rhs.r2Surface;
  }

  bool operator!=(const Contact & rhs) const
  {
    return !(*this == rhs);
  }

  /** Default constructor, invalid contact */
  Contact() = default;

  static Contact from_mc_rbdyn(const Controller &, const mc_rbdyn::Contact &);
};

} // namespace fsm

} // namespace mc_control

namespace std
{

template<>
struct hash<mc_control::fsm::Contact>
{
  std::size_t operator()(const mc_control::fsm::Contact & c) const noexcept
  {
    auto h = std::hash<std::string>{}(c.r1);
    // Same as boost::hash_combine
    auto hash_combine = [&h](const std::string & value) {
      h ^= std::hash<std::string>{}(value) + 0x9e3779b9 + (h << 6) + (h >> 2);
    };
    hash_combine(c.r1Surface);
    hash_combine(c.r2);
    hash_combine(c.r2Surface);
    return h;
  }
};

} // namespace std

namespace mc_control
{

namespace fsm
{

using ContactSet =
    std::unordered_set<Contact, std::hash<Contact>, std::equal_to<Contact>, Eigen::aligned_allocator<Contact>>;

/** \class Controller
 *
 * This controller implements a finite-state machine to handle complex
 * scenarios in a generic fashion.
 *
 * The FSM can be controlled in two ways:
 *
 * - managed: in that case, an external tool triggers transitions
 *
 * - self-managed: the FSM takes care of transition, in that case a
 *   transition map must be provided.
 *
 */
struct MC_CONTROL_FSM_DLLAPI Controller : public MCController
{
  friend struct Executor;

  Controller(std::shared_ptr<mc_rbdyn::RobotModule> rm, double dt, const mc_rtc::Configuration & config);

  ~Controller() override;

  bool run() override;

  /** Can be called in derived class to run the FSM with a different feedback strategy
   *
   * \param fType Type of feedback used in the solver
   *
   */
  bool run(mc_solver::FeedbackType fType);

  void reset(const ControllerResetData & data) override;

  /** Stop the current state's execution
   *
   * The controller will switch to its idle behaviour which maintains current
   * contacts, free-flyer position/orientation and posture.
   *
   * This function is virtual to allow derived implementation to handle
   * interruptions differently.
   */
  virtual void interrupt()
  {
    executor_.interrupt();
  }

  /** Check if current state is running */
  bool running()
  {
    return executor_.running();
  }

  /** Resume the FSM execution on a new state
   *
   * This call is ignored if running() returns true
   *
   * \param state State to start
   *
   * \return True if the state was started
   *
   */
  bool resume(const std::string & state);

  /** Add collisions-pair between two robots
   *
   * If the r1-r2 collision manager does not exist yet, it is created and
   * added to the solver.
   */
  void addCollisions(const std::string & r1,
                     const std::string & r2,
                     const std::vector<mc_rbdyn::Collision> & collisions);

  /** Remove collisions-pair between two robots
   *
   * If the r1-r2 collision manager does not exist yet, this has no
   * effect.
   */
  void removeCollisions(const std::string & r1,
                        const std::string & r2,
                        const std::vector<mc_rbdyn::Collision> & collisions);

  /** Remove all collision-pair between two robots
   *
   * If the r1-r2 collision manager does not exist yet, this has no
   * effect.
   */
  void removeCollisions(const std::string & r1, const std::string & r2);

  /** Returns true if the robot is part of the controller */
  bool hasRobot(const std::string & robot) const;

  using MCController::robot;

  /** Access robot by name
   *
   * \param name Name of the robot
   *
   * \throws If hasRobot(name) returns false
   */
  mc_rbdyn::Robot & robot(const std::string & name);

  /** Get the posture task associated to a robot
   *
   * Returns a nullptr if the task does not exist (i.e. robot is not
   * actuated or not loaded by the controller)
   *
   */
  std::shared_ptr<mc_tasks::PostureTask> getPostureTask(const std::string & robot);

  /** Add a contact between two robots
   *
   * No effect if the contact is already present.
   *
   */
  void addContact(const Contact & c);

  /** Remove a contact between two robots
   *
   * No effect if the contact is already absent.
   *
   */
  void removeContact(const Contact & c);

  /** Access the current contacts */
  const ContactSet & contacts() const;

  /** Check if a contact is already present */
  bool hasContact(const Contact & c) const;

  /** Access contact constraint */
  mc_solver::ContactConstraint & contactConstraint()
  {
    return *contact_constraint_;
  }

  /** Access the state factory */
  StateFactory & factory()
  {
    return factory_;
  }

private:
  /** Reset all posture tasks */
  void resetPostures();

  /** Update the contacts (or their DoFs) if needed */
  void updateContacts();

  /** Start the idle state */
  void startIdleState();

  /** Teardown the idle state */
  void teardownIdleState();

private:
  /** Map robots' names to index */
  std::map<std::string, size_t> robots_idx_;

  /** Holds dynamics, kinematics and contact constraints that are added
   * from the start by the controller */
  std::vector<mc_solver::ConstraintSetPtr> constraints_;

  /** Keep track of the contact constraint */
  std::shared_ptr<mc_solver::ContactConstraint> contact_constraint_ = nullptr;

  /** Collision managers for robot-pair (r1, r2), if r1 == r2 this is
   * effectively a self-collision manager */
  std::map<std::pair<std::string, std::string>, std::shared_ptr<mc_solver::CollisionsConstraint>> collision_constraints_;

  /** Creates a posture task for each actuated robots
   * (i.e. robot.dof() - robot.joint(0).dof() > 0 ) */
  std::map<std::string, std::shared_ptr<mc_tasks::PostureTask>> posture_tasks_;
  std::map<std::string, double> saved_posture_weights_;

  /** Creates a free-flyer end-effector task for each robot with a free flyer */
  std::map<std::string, std::shared_ptr<mc_tasks::EndEffectorTask>> ff_tasks_;

  /** FSM contacts */
  ContactSet contacts_;
  /** True if contacts were changed in the previous round */
  bool contacts_changed_;

  /** State factory */
  StateFactory factory_;

  /** Behaviour during idle */
  bool idle_keep_state_ = false;
  /** True if not idle */
  bool running_ = false;
  /** Main executor */
  Executor executor_;

  using duration_ms = std::chrono::duration<double, std::milli>;
  /** Monitor updateContacts runtime */
  duration_ms updateContacts_dt_{0};
};

} // namespace fsm

} // namespace mc_control

namespace mc_rtc
{

template<>
struct MC_CONTROL_FSM_DLLAPI ConfigurationLoader<mc_control::fsm::Contact>
{
  static mc_control::fsm::Contact load(const mc_rtc::Configuration & config);
};

} // namespace mc_rtc
