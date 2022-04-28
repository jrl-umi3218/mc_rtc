/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/Configuration.h>
#include <mc_control/Contact.h>

#include <mc_observers/ObserverPipeline.h>

#include <mc_rbdyn/Robots.h>

#include <mc_rtc/DataStore.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/log/Logger.h>

#include <mc_solver/CollisionsConstraint.h>
#include <mc_solver/CompoundJointConstraint.h>
#include <mc_solver/ContactConstraint.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/KinematicsConstraint.h>
#include <mc_solver/QPSolver.h>
#include <mc_solver/msg/QPResult.h>

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

  /** Gives access to the result of the QP execution
   * \param t Unused at the moment
   */
  virtual const mc_solver::QPResultMsg & send(const double & t);

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
  bool hasRobot(const std::string & robot) const;

  /** Return the main robot (first robot provided in the constructor)
   * \anchor mc_controller_robot_const_doc
   */
  const mc_rbdyn::Robot & robot() const;

  /** Return the env "robot"
   * \note
   * In multi-robot scenarios, the env robot is either:
   *   1. The first robot with zero dof
   *   2. The last robot provided at construction
   * \anchor mc_controller_env_const_doc
   */
  const mc_rbdyn::Robot & env() const;

  /** Return the mc_rbdyn::Robots controlled by this controller
   * \anchor mc_controller_robots_const_doc
   */
  const mc_rbdyn::Robots & robots() const;

  /** Non-const variant of \ref mc_controller_robots_const_doc "robots()" */
  mc_rbdyn::Robots & robots();

  /** Return the mc_rbdyn::Robot controlled by this controller
   *
   * @throws std::runtime_error if the robot does not exist
   * \anchor mc_controller_robot_name_const_doc
   **/
  const mc_rbdyn::Robot & robot(const std::string & name) const;

  /** Non-const variant of \ref mc_controller_robot_name_const_doc "robot(name)" */
  mc_rbdyn::Robot & robot(const std::string & name);

  /** Non-const variant of \ref mc_controller_robot_const_doc "robot()" */
  mc_rbdyn::Robot & robot();

  /** Non-const variant of \ref mc_controller_env_const_doc "env()" */
  mc_rbdyn::Robot & env();

  /** Return the mc_solver::QPSolver instance attached to this controller
   * \anchor mc_controller_qpsolver_const_doc
   */
  const mc_solver::QPSolver & solver() const;

  /** Non-const variant of \ref mc_controller_qpsolver_const_doc "solver()" */
  mc_solver::QPSolver & solver();

  /** Returns mc_rtc::Logger instance */
  mc_rtc::Logger & logger();

  /** Returns mc_rtc::gui::StateBuilder ptr */
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui()
  {
    return gui_;
  }

  /** Provides access to the shared datastore */
  mc_rtc::DataStore & datastore()
  {
    return datastore_;
  }

  /** Provides access to the shared datastore (const) */
  const mc_rtc::DataStore & datastore() const
  {
    return datastore_;
  }

  /** Return the mc_rbdyn::Robots real robots instance
   * \anchor mc_controller_real_robots_const_doc
   */
  const mc_rbdyn::Robots & realRobots() const;
  /** Non-const variant of \ref mc_controller_real_robots_const_doc "realRobots()" **/
  mc_rbdyn::Robots & realRobots();

  /** Return the main mc_rbdyn::Robot real robot instance
   * \anchor mc_controller_real_robot_const_doc
   */
  const mc_rbdyn::Robot & realRobot() const;
  /** Non-const variant of \ref mc_controller_real_robot_const_doc "realRobot()" */
  mc_rbdyn::Robot & realRobot();

  /** Return the mc_rbdyn::Robot controlled by this controller
   *
   * @throws std::runtime_error if the robot does not exist
   * \anchor mc_controller_realRobot_name_const_doc
   **/
  const mc_rbdyn::Robot & realRobot(const std::string & name) const;

  /** Non-const variant of \ref mc_controller_realRobot_name_const_doc "realRobot(name)" */
  mc_rbdyn::Robot & realRobot(const std::string & name);

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
  mc_rtc::Configuration & config()
  {
    return config_;
  }

  /** Access controller configuration (const) */
  const mc_rtc::Configuration & config() const
  {
    return config_;
  }

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
  /** Builds a controller base with an empty environment
   * \param robot Pointer to the main RobotModule
   * \param dt Controller timestep
   * your controller
   */
  MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt);

  MCController(std::shared_ptr<mc_rbdyn::RobotModule> robot, double dt, const mc_rtc::Configuration & config);

  /** Builds a multi-robot controller base
   * \param robots Collection of robot modules used by the controller
   * \param dt Timestep of the controller
   */
  MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robot_modules, double dt);

  /** Builds a multi-robot controller base
   * \param robots Collection of robot modules used by the controller
   * \param dt Timestep of the controller
   * \param config Controller configuration
   */
  MCController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robot_modules,
               double dt,
               const mc_rtc::Configuration & config);

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
                              bool updateNrVars = true);

  /** Update the contacts (or their DoFs) if needed */
  void updateContacts();

protected:
  /** QP solver */
  std::shared_ptr<mc_solver::QPSolver> qpsolver;

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
  mc_solver::ContactConstraint contactConstraint;
  /** Dynamics constraints for the main robot */
  mc_solver::DynamicsConstraint dynamicsConstraint;
  /** Kinematics constraints for the main robot */
  mc_solver::KinematicsConstraint kinematicsConstraint;
  /** Self collisions constraint for the main robot */
  mc_solver::CollisionsConstraint selfCollisionConstraint;
  /** Compound joint constraint for the main robot */
  std::unique_ptr<mc_solver::CompoundJointConstraint> compoundJointConstraint;
  /** Posture task for the main robot */
  std::shared_ptr<mc_tasks::PostureTask> postureTask;
  /* Controller's name */
  std::string name_;
};

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
