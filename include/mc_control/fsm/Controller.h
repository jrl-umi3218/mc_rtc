#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/TransitionMap.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/PostureTask.h>

namespace mc_control
{

namespace fsm
{

struct Controller;
struct State;

/** \class Contact
 *
 * A lightweight variant of mc_rbdyn::Contact meant to simplify contact
 * manipulation in the FSM
 *
 */
struct MC_CONTROL_DLLAPI Contact
{
  std::string r1;
  std::string r2;
  std::string r1Surface;
  std::string r2Surface;

  bool operator<(const Contact & rhs) const
  {
    return r1 < rhs.r1 ||
           ( r1 == rhs.r1 && r1Surface < rhs.r1Surface ) ||
           ( r1 == rhs.r1 && r1Surface == rhs.r1Surface && r2 < rhs.r2 ) ||
           ( r1 == rhs.r1 && r1Surface == rhs.r1Surface && r2 == rhs.r2 && r2Surface < rhs.r2Surface );
  }

  bool operator==(const Contact & rhs) const
  {
    return r1 == rhs.r1 && r2 == rhs.r2 &&
           r1Surface == rhs.r1Surface && r2Surface == rhs.r2Surface;
  }

  static Contact from_mc_rbdyn(const Controller &, const mc_rbdyn::Contact &);
};

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
struct MC_CONTROL_DLLAPI Controller : public MCController
{
  Controller(std::shared_ptr<mc_rbdyn::RobotModule> rm,
             double dt,
             const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const ControllerResetData & data) override;

  bool play_next_stance() override;

  bool read_msg(std::string & msg) override;

  bool read_write_msg(std::string & msg, std::string & out) override;

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
  void removeCollisions(const std::string & r1,
                        const std::string & r2);

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
  const std::set<Contact> & contacts() const;

  /** Check if a contact is already present */
  bool hasContact(const Contact & c) const;
private:
  /** Reset all posture tasks */
  void resetPostures();

  /** Go to the next state */
  void nextState();
private:
  /** Keep track of the configuration of the controller */
  mc_rtc::Configuration config_;
  /** Map robots' names to index */
  std::map<std::string, size_t> robots_idx_;

  /** If true, transitions are managed by an external tool */
  bool managed_;
  /** If true and the FSM is self-managed, all transitions require a user-input */
  bool step_by_step_;

  /** Holds dynamics, kinematics and contact constraints that are added
   * from the start by the controller */
  std::vector<mc_solver::ConstraintSetPtr> constraints_;

  /** Collision managers for robot-pair (r1, r2), if r1 == r2 this is
   * effectively a self-collision manager */
  std::map<std::pair<std::string, std::string>,
    std::shared_ptr<mc_solver::CollisionsConstraint>> collision_constraints_;

  /** Creates a posture task for each actuated robots
   * (i.e. robot.dof() - robot.joint(0).dof() > 0 ) */
  std::map<std::string, std::shared_ptr<mc_tasks::PostureTask>> posture_tasks_;

  /** Creates a CoM task for each robots with a free-flyer base */
  std::map<std::string, std::shared_ptr<mc_tasks::CoMTask>> com_tasks_;

  /** FSM contacts */
  std::set<Contact> contacts_;
  /** True if contacts were changed in the previous round */
  bool contacts_changed_;

  /** State factory */
  StateFactory factory_;

  /** Current state */
  std::shared_ptr<State> state_ = nullptr;
  /** Current state (name) */
  std::string curr_state_ = "";
  /** State output */
  std::string state_output_ = "";

  /** Transition map, empty when the FSM is managed */
  TransitionMap transition_map_;

  /** If true, the current state is interrupted */
  bool interrupt_triggered_ = false;
  /** If true, transition is triggered */
  bool transition_triggered_ = false;
  /** Name of the next state */
  std::string next_state_ = "";
};

} // namespace fsm

} // namespace mc_control
