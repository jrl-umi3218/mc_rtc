#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/mc_fsm_state_factory.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/PostureTask.h>

namespace mc_control
{

struct FSMState;

/** \class FSMContact
 *
 * A lightweight variant of mc_rbdyn::Contact meant to simplify contact
 * manipulation in the FSM
 *
 */
struct MC_CONTROL_DLLAPI FSMContact
{
  std::string r1;
  std::string r2;
  std::string r1Surface;
  std::string r2Surface;

  bool operator<(const FSMContact & rhs) const
  {
    return r1 < rhs.r1 ||
           ( r1 == rhs.r1 && r1Surface < rhs.r1Surface ) ||
           ( r1 == rhs.r1 && r1Surface == rhs.r1Surface && r2 < rhs.r2 ) ||
           ( r1 == rhs.r1 && r1Surface == rhs.r1Surface && r2 == rhs.r2 && r2Surface < rhs.r2Surface );
  }
};

/** \class FSMController
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
 * A transition map is formed by entries of the form ["StateName",
 * "OutputName", "NewStateName", "OptionalTriggerType"], e.g.:
 * - ["InitState", "OK", "GraspBar", "Strict"]
 * - ["GraspBar", "OK", "LiftLeftFoot", "StepByStep"]
 * - ["GraspBar", "NOK", "GraspBar", "Auto"]
 *
 * Valid values for the trigger type are:
 * - "StepByStep": only require user input if running in StepByStep mode
 *   (default)
 * - "Auto": automatic transition no matter what
 * - "Strict": require user input no matter what
 *
 * When such a transition map is provided, the FSM controller will make
 * sure that the map is coherent.
 *
 */
struct MC_CONTROL_DLLAPI FSMController : public MCController
{
  FSMController(std::shared_ptr<mc_rbdyn::RobotModule> rm,
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
  void addContact(const FSMContact & c);

  /** Remove a contact between two robots
   *
   * No effect if the contact is already absent.
   *
   */
  void removeContact(const FSMContact & c);

  /** Access the current contacts */
  const std::set<FSMContact> & contacts() const;
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
  std::set<FSMContact> contacts_;
  /** True if contacts were changed in the previous round */
  bool contacts_changed_;

  /** State factory */
  FSMStateFactory factory_;

  /** Current state */
  std::shared_ptr<FSMState> state_ = nullptr;
  /** Current state (name) */
  std::string curr_state_ = "";
  /** State output */
  std::string state_output_ = "";

  struct Transition
  {
    std::string state;
    enum struct Type
    {
      StepByStep,
      Auto,
      Strict
    };
    Type type;
  };

  struct TransitionMap
  {
    /** A (state, output) pair is the origin of a transition */
    using origin_t = std::pair<std::string, std::string>;

    /** Return a transition given a current state and its ouput
     *
     * \param state The current state
     *
     * \param output The state's output
     *
     * \returns A pair made of a bool and a Transition. If the (state,
     * output) has a registered next state, the bool is true and Transition
     * returns the corresponding transition. Otherwise, the bool is false
     * and the Transition has no meaning.
     *
     */
     std::pair<bool, Transition> transition(const std::string & state,
                                            const std::string & output) const;

    /** Build the map from a Configuration */
    void init(const FSMStateFactory & factory,
              const mc_rtc::Configuration & config);

    /** Returns the initial state value */
    const std::string & initState() const;

    /** Print the map */
    std::ostream & print(std::ostream & os) const;
  private:
    std::string init_state_;
    std::map<origin_t, Transition> map_;
  };
  /** Transition map, empty when the FSM is managed */
  TransitionMap transition_map_;

  /** If true, the current state is interrupted */
  bool interrupt_triggered_ = false;
  /** If true, transition is triggered */
  bool transition_triggered_ = false;
  /** Name of the next state */
  std::string next_state_ = "";
};

}
