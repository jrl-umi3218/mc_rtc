/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/Configuration.h>
#include <mc_control/Contact.h>

#include <mc_observers/ObserverPipeline.h>

#include <mc_rbdyn/RobotConverter.h>
#include <mc_rbdyn/Robots.h>

#include <mc_rtc/DataStore.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/unique_ptr.h>

#include <mc_solver/CollisionsConstraint.h>
#include <mc_solver/CompoundJointConstraint.h>
#include <mc_solver/ContactConstraint.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/KinematicsConstraint.h>
#include <mc_solver/QPSolver.h>

#include <mc_tasks/PostureTask.h>

namespace mc_rbdyn
{
struct Contact;
}

#include <mc_control/api.h>

namespace mc_control
{

/** \class ControllerResetData
 * \brief Contains information allowing the controller to start smoothly from
 * the current state of the robot
 * \note
 * For now, this only contains the state of the robot (free flyer and joints state)
 */
struct MC_CONTROL_DLLAPI ControllerResetData
{
  /** Contains free flyer + joints state information */
  const std::vector<std::vector<double>> q;
};

/** \brief Extra parameters that influence the creator construction */
struct MC_CONTROL_DLLAPI ControllerParameters
{
  inline ControllerParameters() = default;
  inline ControllerParameters(const ControllerParameters &) = default;
  inline ControllerParameters(ControllerParameters &&) = default;
  inline ControllerParameters & operator=(const ControllerParameters &) = default;
  inline ControllerParameters & operator=(ControllerParameters &&) = default;

#define ADD_PARAMETER(TYPE, NAME, DEFAULT) \
  TYPE NAME##_ = DEFAULT;                  \
  ControllerParameters & NAME(TYPE value)  \
  {                                        \
    NAME##_ = value;                       \
    return *this;                          \
  }
  /** Backend used by this controller */
  ADD_PARAMETER(mc_solver::QPSolver::Backend, backend, mc_solver::QPSolver::Backend::Tasks)
  /** Whether to automatically load the robots' specific configuration (true by default) */
  ADD_PARAMETER(bool, load_robot_config, true)
  /** If true, the robot's config is loaded directly into the provided section, otherwise it is nested under the robot's
   * module name */
  ADD_PARAMETER(bool, overwrite_config, false)
  /** Use the module name to find robot's specific values if true (default), use the robot's name otherwise */
  ADD_PARAMETER(bool, load_robot_config_with_module_name, true)
  /** Where to load the robots' configuration, config("robots") by default */
  ADD_PARAMETER(std::vector<std::string>, load_robot_config_into, {"robots"})
  /** Extra configuration file to look for and load into the specified section, furthermore:
   * - the search path matches robots' configuration files
   * - they are loaded after the robots' configuration files provided at construction and before the robots' specific
   * file for dynamically specified robots, this is done to allow extra configuration files to load new robots when \ref
   * overwrite_config is true and the configuration are loaded into the root
   */
  ADD_PARAMETER(std::vector<std::string>, extra_configurations, {})

  /** For backward compatibility purpose */
  inline ControllerParameters(mc_solver::QPSolver::Backend backend) : backend_(backend) {}
};

struct MCGlobalController;

/** \class MCController
 * \brief MCController is the base class to implement all controllers. It
 * assumes that at least two robots are provided. The first is considered as the
 * "main" robot. Some common constraints and a posture task are defined (but not
 * added to the solver) for this robot
 */
struct MC_CONTROL_DLLAPI MCController
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend struct MCGlobalController;

public:
  /** Shortcut for the Backend enum type */
  using Backend = mc_solver::QPSolver::Backend;

  virtual ~MCController();
  /** This function is called at each time step of the process driving the robot
   * (i.e. simulation or robot's controller). This function is the most likely
   * to be overriden for complex controller behaviours.
   * \return True if the solver succeeded, false otherwise
   * \note
   * This is meant to run in real-time hence some precaution should apply (e.g.
   * no i/o blocking calls, no thread instantiation and such)
   *
   * \note
   * The default implementation does the bare minimum (i.e. call run on QPSolver)
   * It is recommended to use it in your override.
   */
  virtual bool run();

  /**
   * Create state observation pipelines from configuration
   *
   * Please refer to the ObserverPipelines JSON Schema for supported
   * configuration options.
   *
   * \see mc_observers::ObserverPipeline
   **/
  virtual void createObserverPipelines(const mc_rtc::Configuration & config);

  /** This function is called before the run() function at each time step of the process
   * driving the robot (i.e. simulation or robot's controller). The default
   * behaviour is to call the run() function of each loaded observer and update
   * the realRobot instance when desired.
   *
   * This is meant to run in real-time hence some precaution should apply (e.g.
   * no i/o blocking calls, no thread instantiation and such)
   *
   * \note Some estimators are likely to require extra information from the control.
   * Each observer has access to the `MCController` instance, and may access all information
   * available from it (robots, etc). In addition some observers may require
   * additional information that is not part of the `MCController` instance. In
   * that case, it may be provided though the `Datastore` (see each observer's
   * documentation for specific requirements).
   *
   * \note If the default pipeline behaviour does not suit you, you may override
   * this method.
   *
   * @returns true if all observers ran as expected, false otherwise
   */
  virtual bool runObserverPipelines();

  /*! @brief Reset the observers.
   *
   * This function is called after the reset() function.
   *
   * @returns True when all observers have been succesfully reset.
   */
  virtual bool resetObserverPipelines();

  /** Whether this controller contains a pipeline with the provided name
   *
   * \param name Name of the pipeline
   */
  bool hasObserverPipeline(const std::string & name) const;

  /** True if this controller has at least one state observation pipeline */
  bool hasObserverPipeline() const;

  /**
   * Provides const access to the state observation pipelines defined in this
   * controller
   */
  const std::vector<mc_observers::ObserverPipeline> & observerPipelines() const;
  /** Non-const variant */
  std::vector<mc_observers::ObserverPipeline> & observerPipelines();

  /**
   * Provides const access to a state-observation pipeline
   *
   * @throws if no pipeline with that name exist
   */
  const mc_observers::ObserverPipeline & observerPipeline(const std::string & name) const;
  /** Non-const variant */
  mc_observers::ObserverPipeline & observerPipeline(const std::string & name);

  /**
   * Provides const access to the main observer pipeline (first pipeline)
   *
   * @throws if this controller does not have any pipeline
   */
  const mc_observers::ObserverPipeline & observerPipeline() const;
  /** Non-const variant */
  mc_observers::ObserverPipeline & observerPipeline();

  /** Can be called in derived class instead of run to use a feedback strategy
   * different from the default one
   *
   * \param fType Type of feedback used in the solver
   *
   */
  bool run(mc_solver::FeedbackType fType);

  /** This function is called when the controller is stopped.
   *
   * The default implementation does nothing.
   *
   * For example, it can be overriden to signal threads launched by the
   * controller to pause.
   */
  virtual void stop();

  /** Reset the controller with data provided by ControllerResetData. This is
   * called at two possible points during a simulation/live execution:
   *   1. Actual start
   *   2. Switch from a previous (MCController-like) controller
   * In the first case, the data comes from the simulation/controller. In the
   * second case, the data comes from the previous MCController instance.
   * \param reset_data Contains information allowing to reset the controller
   * properly
   * \note
   * The default implementation reset the main robot's state to that provided by
   * reset_data (with a null speed/acceleration). It maintains the contacts as
   * they were set by the controller previously.
   *
   * \throws if the main robot is not supported (see supported_robots())
   */
  virtual void reset(const ControllerResetData & reset_data);

  /** Add collisions-pair between two robots
   *
   * If the r1-r2 collision manager does not exist yet, it is created and
   * added to the solver.
   */
  void addCollisions(const std::string & r1,
                     const std::string & r2,
                     const std::vector<mc_rbdyn::Collision> & collisions);

  /** Returns true if the given collision is active */
  bool hasCollision(const std::string & r1, const std::string & r2, const mc_rbdyn::Collision & col) const noexcept;

  /** Returns true if the given collision is active */
  bool hasCollision(const std::string & r1,
                    const std::string & r2,
                    const std::string & c1,
                    const std::string & c2) const noexcept;

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

  /** Remove all contacts */
  void clearContacts();

  /** Access the current contacts */
  const ContactSet & contacts() const;

  /** Check if a contact is already present */
  bool hasContact(const Contact & c) const;

  /** Returns true if the robot is part of the controller */
  inline bool hasRobot(const std::string & robot) const noexcept { return robots().hasRobot(robot); }

  /** Return the mc_rbdyn::Robots controlled by this controller
   * \anchor mc_controller_robots_const_doc
   */
  inline const mc_rbdyn::Robots & robots() const noexcept { return solver().robots(); }

  /** Non-const variant of \ref mc_controller_robots_const_doc "robots()" */
  inline mc_rbdyn::Robots & robots() noexcept { return solver().robots(); }

  /**
   * @name Accessors to the control robots
   * @{
   */
  /** Return the main robot (first robot provided in the constructor)
   * \anchor mc_controller_robot_const_doc
   */
  inline const mc_rbdyn::Robot & robot() const noexcept { return robots().robot(); }

  /** Non-const variant of \ref mc_controller_robot_const_doc "robot()" */
  inline mc_rbdyn::Robot & robot() noexcept { return robots().robot(); }

  /** Return the mc_rbdyn::Robot controlled by this controller
   *
   * @throws std::runtime_error if the robot does not exist
   * \anchor mc_controller_robot_name_const_doc
   **/
  inline const mc_rbdyn::Robot & robot(const std::string & name) const { return robots().robot(name); }

  /** Non-const variant of \ref mc_controller_robot_name_const_doc "robot(name)" */
  inline mc_rbdyn::Robot & robot(const std::string & name) { return robots().robot(name); }

  /** Return the env "robot"
   * \note
   * In multi-robot scenarios, the env robot is either:
   *   1. The first robot with zero dof
   *   2. The last robot provided at construction
   * \anchor mc_controller_env_const_doc
   */
  inline const mc_rbdyn::Robot & env() const noexcept { return robots().env(); }

  /** Non-const variant of \ref mc_controller_env_const_doc "env()" */
  inline mc_rbdyn::Robot & env() noexcept { return robots().env(); }
  /** @} */

  /** Return the mc_solver::QPSolver instance attached to this controller
   * \anchor mc_controller_qpsolver_const_doc
   */
  inline const mc_solver::QPSolver & solver() const noexcept { return *qpsolver; }

  /** Non-const variant of \ref mc_controller_qpsolver_const_doc "solver()" */
  inline mc_solver::QPSolver & solver() noexcept { return *qpsolver; }

  /** Returns mc_rtc::Logger instance */
  inline mc_rtc::Logger & logger() noexcept { return *logger_; }

  /** Returns mc_rtc::gui::StateBuilder ptr */
  inline std::shared_ptr<mc_rtc::gui::StateBuilder> gui() const noexcept { return gui_; }

  /** Provides access to the shared datastore */
  inline mc_rtc::DataStore & datastore() noexcept { return datastore_; }

  /** Provides access to the shared datastore (const) */
  const mc_rtc::DataStore & datastore() const noexcept { return datastore_; }

  /**
   * @name Accessors to the real robots
   * @{
   */
  /** Return the mc_rbdyn::Robots real robots instance
   * \anchor mc_controller_real_robots_const_doc
   */
  inline const mc_rbdyn::Robots & realRobots() const noexcept { return solver().realRobots(); }
  /** Non-const variant of \ref mc_controller_real_robots_const_doc "realRobots()" **/
  inline mc_rbdyn::Robots & realRobots() noexcept { return solver().realRobots(); }

  /** Return the main mc_rbdyn::Robot real robot instance
   * \anchor mc_controller_real_robot_const_doc
   */
  inline const mc_rbdyn::Robot & realRobot() const noexcept { return realRobots().robot(); }
  /** Non-const variant of \ref mc_controller_real_robot_const_doc "realRobot()" */
  inline mc_rbdyn::Robot & realRobot() noexcept { return realRobots().robot(); }

  /** Return the mc_rbdyn::Robot controlled by this controller
   *
   * @throws std::runtime_error if the robot does not exist
   * \anchor mc_controller_realRobot_name_const_doc
   **/
  inline const mc_rbdyn::Robot & realRobot(const std::string & name) const { return realRobots().robot(name); }

  /** Non-const variant of \ref mc_controller_realRobot_name_const_doc "realRobot(name)" */
  inline mc_rbdyn::Robot & realRobot(const std::string & name) { return realRobots().robot(name); }
  /** @} */

  /**
   * @name Accessors to the output robots
   *
   * Output robots should be used by interface and plugin that need to get a view of the complete system being
   * controlled.
   * @{
   */
  /** Return the output robots
   * \anchor mc_controller_real_robots_const_doc
   */
  inline const mc_rbdyn::Robots & outputRobots() const noexcept { return *outputRobots_; }
  /** Non-const variant of \ref mc_controller_output_robots_const_doc "outputRobots()" **/
  inline mc_rbdyn::Robots & outputRobots() noexcept { return *outputRobots_; }

  /** Return the main robot's output instance
   * \anchor mc_controller_output_robot_const_doc
   */
  inline const mc_rbdyn::Robot & outputRobot() const noexcept { return outputRobots_->robot(); }
  /** Non-const variant of \ref mc_controller_output_robot_const_doc "outputRobot()" */
  inline mc_rbdyn::Robot & outputRobot() noexcept { return outputRobots_->robot(); }

  /** Return an output robot by name
   *
   * @throws std::runtime_error if the robot does not exist
   * \anchor mc_controller_outputRobot_name_const_doc
   **/
  inline const mc_rbdyn::Robot & outputRobot(const std::string & name) const { return outputRobots_->robot(name); }

  /** Non-const variant of \ref mc_controller_outputRobot_name_const_doc "outputRobot(name)" */
  inline mc_rbdyn::Robot & outputRobot(const std::string & name) { return outputRobots_->robot(name); }
  /** @} */

  /**
   * @name Accessors to the output real robots
   *
   * These robots are used by the various interfaces to send control commands to
   * the robots, and by the ROS plugin to publish a complete visualization of
   * the robots (including non-controlled joints)
   * @{
   */
  /** Return the mc_rbdyn::Robots real robots instance
   * \anchor mc_controller_output_real_robots_const_doc
   */
  inline const mc_rbdyn::Robots & outputRealRobots() const noexcept { return *outputRealRobots_; }
  /** Non-const variant of \ref mc_controller_output_real_robots_const_doc "outputRealRobots()" **/
  inline mc_rbdyn::Robots & outputRealRobots() noexcept { return *outputRealRobots_; }

  /** Return the main mc_rbdyn::Robot real robot instance
   * \anchor mc_controller_output_real_robot_const_doc
   */
  inline const mc_rbdyn::Robot & outputRealRobot() const noexcept { return outputRealRobots_->robot(); }
  /** Non-const variant of \ref mc_controller_output_real_robot_const_doc "outputRealRobot()" */
  inline mc_rbdyn::Robot & outputRealRobot() noexcept { return outputRealRobots_->robot(); }

  /** Return the mc_rbdyn::Robot controlled by this controller
   *
   * @throws std::runtime_error if the robot does not exist
   * \anchor mc_controller_outputRealRobot_name_const_doc
   **/
  inline const mc_rbdyn::Robot & outputRealRobot(const std::string & name) const
  {
    return outputRealRobots_->robot(name);
  }

  /** Non-const variant of \ref mc_controller_outputRealRobot_name_const_doc "outputRealRobot(name)" */
  inline mc_rbdyn::Robot & outputRealRobot(const std::string & name) { return outputRealRobots_->robot(name); }
  /** @} */

  /** Returns a list of robots supported by the controller.
   * \param out Vector of supported robots designed by name (as returned by
   * RobotModule::name())
   * \note
   * - Default implementation returns an empty list which indicates that all
   * robots are supported.
   * - If the list is not empty, only the robots in that list are allowed to be
   *   used with the controller. The main robot will be checked against the list of supported robots
   * upon call to reset(), and an exception will be thrown if the robot is not supported.
   */
  virtual void supported_robots(std::vector<std::string> & out) const;

  /** Load an additional robot into the controller (and its corresponding
   * realRobot instance)
   *
   * \param rm RobotModule used to load the robot
   *
   * \param name Name of the robot
   *
   * \return The loaded control robot.
   * You may access the corresponding real robot through realRobots().robot(name)
   */
  mc_rbdyn::Robot & loadRobot(mc_rbdyn::RobotModulePtr rm, const std::string & name);

  /** Remove a robot from the controller
   *
   * \param name Name of the robot to remove
   */
  void removeRobot(const std::string & name);

  /** Access or modify controller configuration */
  mc_rtc::Configuration & config() { return config_; }

  /** Access controller configuration (const) */
  const mc_rtc::Configuration & config() const { return config_; }

  /** Access a gripper by robot's name and gripper's name
   *
   * \param robot Name of the robot
   *
   * \param gripper Name of the gripper
   *
   * \throws If the robot's name is not valid or the gripper's name is not valid
   */
  Gripper & gripper(const std::string & robot, const std::string & gripper);

  /** Helper to make the anchor frame compile-time deprecation warning
   * clearer
   */
  template<typename T>
  struct DeprecatedAnchorFrame : public std::false_type
  {
  };

  /** @deprecated The observer's anchor frame is now provided by a callback on
   * the datastore. For further information, please refer to the observer's tutorial:
   * https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html
   */
  template<typename T = void> // void for shorter error message
  inline void anchorFrame(const sva::PTransformd &)
  {
    static_assert(DeprecatedAnchorFrame<T>::value,
                  "[MC_RTC_DEPRECATED] The anchorFrame and anchorFrameReal accessors are no longer supported, please "
                  "remove calls to these functions from your code and replace it with a datastore callback. For "
                  "further information please refer to the observer's tutorial: "
                  "https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html");
  }

  /** @deprecated The observer's anchor frame is now provided by a callback on
   * the datastore. For further information, please refer to the observer's
   * tutorial: https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html
   */
  template<typename T = void> // void for shorter error message
  inline void anchorFrameReal(const sva::PTransformd &)
  {
    static_assert(DeprecatedAnchorFrame<T>::value,
                  "[MC_RTC_DEPRECATED] The anchorFrame and anchorFrameReal accessors are no longer supported, please "
                  "remove calls to these functions from your code and replace it with a datastore callback. For "
                  "further information please refer to the observer's tutorial: "
                  "https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html");
  }
  /** @deprecated The observer's anchor frame is now provided by a callback on
   * the datastore. For further information, please refer to the observer's
   * tutorial: https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html
   */
  template<typename T = void> // void for shorter error message
  inline const sva::PTransformd & anchorFrame() const
  {
    static_assert(DeprecatedAnchorFrame<T>::value,
                  "[MC_RTC_DEPRECATED] The anchorFrame and anchorFrameReal accessors are no longer supported, please "
                  "remove calls to these functions from your code and replace it with a datastore callback. For "
                  "further information please refer to the observer's tutorial: "
                  "https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html");
    return robot().posW();
  }

  /** @deprecated The observer's anchor frame is now provided by a callback on
   * the datastore. For further information, please refer to the observer's
   * tutorial: https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html
   */
  template<typename T = void> // void for shorter error message
  inline const sva::PTransformd & anchorFrameReal() const
  {
    static_assert(DeprecatedAnchorFrame<T>::value,
                  "[MC_RTC_DEPRECATED] The anchorFrame and anchorFrameReal accessors are no longer supported, please "
                  "remove calls to these functions from your code and replace it with a datastore callback. For "
                  "further information please refer to the observer's tutorial: "
                  "https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html");
    return robot().posW();
  }

protected:
  /** Builds a controller with a single robot. The env/ground environment is automatically added
   *
   * \param robot Pointer to the main RobotModule
   *
   * \param dt Timestep of the controller
   *
   * \param params Extra-parameters for the constructor
   */
  MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt, ControllerParameters params = {});

  /** Builds a controller with a single robot. The env/ground environment is automatically added
   *
   * \param robot Pointer to the main RobotModule
   *
   * \param dt Timestep of the controller
   *
   * \param config Configuration of the controller
   *
   * \param params Extra-parameters for the constructor
   */
  MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot,
               double dt,
               const mc_rtc::Configuration & config,
               ControllerParameters params = {});

  /** Builds a controller with multiple robots
   *
   * \param robots Collection of robot modules used by the controller
   *
   * \param dt Timestep of the controller
   *
   * \param params Extra-parameters for the constructor
   */
  MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robot_modules,
               double dt,
               ControllerParameters params = {});

  /** Builds a controller with multiple robots
   *
   * \param robots Collection of robot modules used by the controller
   *
   * \param dt Timestep of the controller
   *
   * \param config Controller configuration
   *
   * \param params Extra-parameters for the constructor
   */
  MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robot_modules,
               double dt,
               const mc_rtc::Configuration & config,
               ControllerParameters params = {});

  /** Load an additional robot into the controller
   *
   * \param name Name of the robot
   * \param rm RobotModule used to load the robot
   * \param robots Robots in which this robot will be loaded
   * \param updateNrVars When true, update the number of variables in the QP
   * problem.
   *
   * \returns The loaded robot
   */
  mc_rbdyn::Robot & loadRobot(mc_rbdyn::RobotModulePtr rm,
                              const std::string & name,
                              mc_rbdyn::Robots & robots,
                              const mc_rbdyn::LoadRobotParameters & params);

  /** Add a control robot to the log */
  void addRobotToLog(const mc_rbdyn::Robot & robot);

  /** Add a control robot to the GUI */
  void addRobotToGUI(const mc_rbdyn::Robot & robot);

  /** Update the contacts (or their DoFs) if needed */
  void updateContacts();

protected:
  /** QP solver */
  std::shared_ptr<mc_solver::QPSolver> qpsolver;

  /** Load robots used for output (display/control)
   *
   * These robots use the canonical model representation defined in the robot
   * module.
   */
  mc_rbdyn::RobotsPtr outputRobots_;
  mc_rbdyn::RobotsPtr outputRealRobots_;
  /** Control to canonical converters */
  std::vector<mc_rbdyn::RobotConverter> converters_;

  /** State observation pipelines for this controller */
  std::vector<mc_observers::ObserverPipeline> observerPipelines_;

  /** Logger provided by MCGlobalController */
  std::shared_ptr<mc_rtc::Logger> logger_;
  /** GUI state builder */
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_;

  /** Keep track of the configuration of the controller */
  mc_rtc::Configuration config_;

  /** DataStore to share variables/objects between different parts of the
   * framework (states...) */
  mc_rtc::DataStore datastore_;

  /** Holds dynamics, kinematics and contact constraints that are added
   * from the start by the controller */
  std::vector<mc_solver::ConstraintSetPtr> constraints_;

  /** Keep track of the contact constraint */
  std::shared_ptr<mc_solver::ContactConstraint> contact_constraint_ = nullptr;

  /** Collision managers for robot-pair (r1, r2), if r1 == r2 this is
   * effectively a self-collision manager */
  std::map<std::pair<std::string, std::string>, std::shared_ptr<mc_solver::CollisionsConstraint>> collision_constraints_;

  /** FSM contacts */
  ContactSet contacts_;
  /** True if contacts were changed in the previous round */
  bool contacts_changed_;
  /** Data shown in the contacts' table: R1, R1Surface, R2, R2Surface, DoF, Friction */
  using ContactTableDataT = std::tuple<std::string, std::string, std::string, std::string, std::string, double>;
  /** Used in GUI display */
  std::vector<ContactTableDataT> contacts_table_;

  using duration_ms = std::chrono::duration<double, std::milli>;
  /** Monitor updateContacts runtime */
  duration_ms updateContacts_dt_{0};

public:
  /** Controller timestep */
  const double timeStep;
  /** Contact constraint for the main robot */
  mc_rtc::unique_ptr<mc_solver::ContactConstraint> contactConstraint;
  /** Dynamics constraints for the main robot */
  mc_rtc::unique_ptr<mc_solver::DynamicsConstraint> dynamicsConstraint;
  /** Kinematics constraints for the main robot */
  mc_rtc::unique_ptr<mc_solver::KinematicsConstraint> kinematicsConstraint;
  /** Self collisions constraint for the main robot */
  mc_rtc::unique_ptr<mc_solver::CollisionsConstraint> selfCollisionConstraint;
  /** Compound joint constraint for the main robot */
  mc_rtc::unique_ptr<mc_solver::CompoundJointConstraint> compoundJointConstraint;
  /** Posture task for the main robot */
  std::shared_ptr<mc_tasks::PostureTask> postureTask;
  /* Controller's name */
  const std::string name_;
  /** Stores the loading location provided by the loader via \ref set_loading_location */
  const std::string loading_location_;

  /** Load a robot specific configuration (if any)
   *
   * The following files are loaded (in that order):
   * - ${loading_location}/${name()}/${robot_name}.conf|yaml
   * - ${HOME}/.config/mc_rtc/controllers/${name()}/${robot_name}.conf|yaml (Linux/macOS)
   * - ${APPDATA}/mc_rtc/controllers/${name()}/${robot_name}.conf|yaml (Windows)
   */
  mc_rtc::Configuration robot_config(const std::string & robot_name) const;

  /** Same as robot_config(robot.module().name) */
  mc_rtc::Configuration robot_config(const mc_rbdyn::Robot & robot) const;

  /** Called by \ref mc_rtc::ObjectLoader to inform the controller of its loading location
   * For example, if the CoM controller is loaded from the library in /usr/local/lib/mc_controller/com.so then this is
   * /usr/local/lib/mc_controller/
   *
   * The value is stored in a thread_local variable and is meant to be used in the constructor of MCController
   */
  static void set_loading_location(std::string_view location);

  /** Called by \ref mc_rtc::ObjectLoader to set the name of the controller
   *
   * The value is stored in a thread_local variable and is meant to be used in the constructor of MCController
   */
  static void set_name(std::string_view name);
};

namespace details
{

/** Helper to declare backend-specific controllers
 *
 * The difference with the default MCController class are:
 * - the backend is always the one specified here
 * - solver() returns the solver class specified here
 */
template<MCController::Backend backend, typename SolverT>
struct BackendSpecificController : public MCController
{
  BackendSpecificController(mc_rbdyn::RobotModulePtr robot, double dt, const mc_rtc::Configuration & config = {})
  : MCController(robot, dt, config, backend)
  {
  }

  BackendSpecificController(const std::vector<mc_rbdyn::RobotModulePtr> & robots,
                            double dt,
                            const mc_rtc::Configuration & config = {},
                            ControllerParameters params = {})
  : MCController(robots, dt, config, params.backend(backend))
  {
  }

  const SolverT & solver() const noexcept { return SolverT::from_solver(MCController::solver()); }

  SolverT & solver() noexcept { return SolverT::from_solver(MCController::solver()); }
};

} // namespace details

} // namespace mc_control

#ifdef WIN32
#  define CONTROLLER_MODULE_API __declspec(dllexport)
#else
#  if __GNUC__ >= 4
#    define CONTROLLER_MODULE_API __attribute__((visibility("default")))
#  else
#    define CONTROLLER_MODULE_API
#  endif
#endif
