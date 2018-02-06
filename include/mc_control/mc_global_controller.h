
#pragma once

#include <mc_control/mc_controller.h>
#include <mc_rtc/log/Logger.h>

#include <mc_control/api.h>

#include <mc_rbdyn/RobotModule.h>

#include <mc_rtc/loader.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <array>
#include <fstream>
#include <sstream>
#include <thread>

namespace mc_control
{

struct MC_CONTROL_DLLAPI MCGlobalController
{
private:
  struct GlobalConfiguration;

  /* MCGlobalController is non-copyable */
  MCGlobalController(const MCGlobalController&) = delete;
  MCGlobalController& operator=(const MCGlobalController&) = delete;

  /* MCGlobalController is non-movable */
  MCGlobalController(MCGlobalController&&) = delete;
  MCGlobalController& operator=(MCGlobalController&&) = delete;

public:

  /*! \brief Create the global controller
   *
   * The global controller is in charge of handling incoming data
   * (sensors, encoders...), passing it to the Controller instance and
   * finally hand out the result to the simulation/robot.
   *
   * An additional configuration file can be passed at construction but
   * multiple configuration files are read by default:
   * - a global configuration file read from
   *   INSTALL_PREFIX/etc/mc_rtc.conf
   * - a global configuration file read from
   *   $HOME/.config/mc_rtc/mc_rtc.conf
   *
   * The configuration provided overrides settings in the local
   * configuration which overrides settings in the global configuration
   *
   * Global settings include the controller timestep, the main robot,
   * publication and logging options.
   *
   * For each controller a specific setting file will be loaded from:
   * - $CONTROLLER_INSTALL_PREFIX/etc/$CONTROLLER_NAME.conf
   * - $HOME/.config/mc_rtc/controllers/$CONTROLLER_NAME.conf
   *
   * Global settings found in controller-specific configuration files
   * will be discarded.
   *
   * \note On Windows, user configuration files are retrieved from
   * $APPDATA/mc_rtc
   *
   * \param conf Additional configuration to load
   *
   */
  MCGlobalController(const std::string & conf = "",
                     std::shared_ptr<mc_rbdyn::RobotModule> rm = nullptr);

  /*! \brief Destructor */
  virtual ~MCGlobalController();

  /*! \brief Returns a list of enabled controllers */
  std::vector<std::string> enabled_controllers() const;

  /*! \brief Returns a list of all the loaded controllers, whether they
   * are enabled or not.
   */
  std::vector<std::string> loaded_controllers() const;

  /*! \brief Returns a list of all the loaded robots, whether they
   * are enabled or not.
   */
  std::vector<std::string> loaded_robots() const;

  /*! \brief Returns the main robot module */
  std::shared_ptr<mc_rbdyn::RobotModule> get_robot_module();

  /*! \brief Returns the name of the current controller */
  std::string current_controller() const;

  /*! \brief Initialize the default controller with the given configuration
   *
   * In this version, the robot's initial attitude is provided by
   * mc_rbdyn::RobotModule
   *
   * \param initq Initial joint configuration
   *
   */
  void init(const std::vector<double> & initq);

  /*! \brief Initialize the default controller with the given configuration and attitude
   *
   * \param initq Initial joint configuration
   *
   * \param initAttitude Initial attitude (qw, qx, qy, qz, tx, ty, tz)
   *
   */
  void init(const std::vector<double> & initq, const std::array<double, 7> & initAttitude);

  /** @name Sensing
   *
   * These functions are used to communicate sensors' information to the
   * controller
   *
   * @{
   */

  /*! \brief A robot's position given by a sensor */
  void setSensorPosition(const Eigen::Vector3d & pos);

  /*! \brief Set multiple body sensors' position for a given robot */
  void setSensorPositions(mc_rbdyn::Robot & robot,
                          const std::map<std::string, Eigen::Vector3d> & poses);

  /*! \brief A robot's orientation given by a sensor */
  void setSensorOrientation(const Eigen::Quaterniond & ori);

  /*! \brief Set multiple body sensors' orientation for a given robot */
  void setSensorOrientations(mc_rbdyn::Robot & robot,
                             const std::map<std::string, Eigen::Quaterniond> & oris);

  /*! \brief A robot's linear velocity given by a sensor */
  void setSensorLinearVelocity(const Eigen::Vector3d& vel);

  /*! \brief Set multiple body sensor's linear velocities for a given robot */
  void setSensorLinearVelocities(mc_rbdyn::Robot & robot,
                                 const std::map<std::string, Eigen::Vector3d> & linearVels);

  /*! \brief A robot's angular velocity given by a sensor */
  void setSensorAngularVelocity(const Eigen::Vector3d& vel);

  /*! \brief Set multiple body sensor's angular velocities for a given robot */
  void setSensorAngularVelocities(mc_rbdyn::Robot & robot,
                                  const std::map<std::string, Eigen::Vector3d> & angularVels);

  /*! \brief A robot's acceleration given by a sensor */
  void setSensorAcceleration(const Eigen::Vector3d& acc);

  /*! \brief Set multiple body sensors' acceleration for a given robot */
  void setSensorAccelerations(mc_rbdyn::Robot & robot,
                              const std::map<std::string, Eigen::Vector3d> & accels);

  /*! \brief A robot's actual joints' values provided by encoders
   *
   * \note It is expected that these values follow the order given by
   * ref_joint_order
   */
  void setEncoderValues(const std::vector<double> & eValues);

  /*! \brief A robot's actual joints' torques provided by sensors
   *
   * \note It is expected that these values follow the order given by
   * ref_joint_order
   */
  void setJointTorques(const std::vector<double> & tValues);

  /*! \brief Force sensors' readings provided by the sensors */
  void setWrenches(const std::map<std::string, sva::ForceVecd> & wrenches);

  /*! \brief Force sensors' readings for another robot than the main robot */
  void setWrenches(unsigned int robotIndex, const std::map<std::string, sva::ForceVecd> & wrenches);

  /*! \brief Gripper active joints actual values */
  void setActualGripperQ(const std::map<std::string, std::vector<double>> & grippersQ);

  /** @} */

  /*! \brief Runs one step of the controller
   *
   * \returns True if the current controller succeeded, false otherwise
   */
  bool run();

  /*! \brief Access the result of the latest run
   *
   * \param t Unused
   *
   * \returns A reference to the latest result
   */
  const mc_solver::QPResultMsg & send(const double & t);

  /*! \brief Access the current controller */
  MCController & controller();

  /*! \brief Access the main robot */
  mc_rbdyn::Robot & robot();

  /*! \brief Get the controller timestep */
  double timestep();

  /*! \brief Access the reference joint order
   *
   * This is provided by mc_rbdyn::RobotModule and represents the joint's order
   * in the native control system of the robot.
   */
  const std::vector<std::string> & ref_joint_order();

  /*! \brief Access the global controller configuration */
  const GlobalConfiguration & configuration() const;

  /*! \brief Add the given directories to the controller search path
   *
   * Calling this function with the same arguments multiple times
   * effectively refresh the loaded controller list
   *
   * e.g. the following code will rescan all directories
   *
   * add_controller_module_paths(configuration().controller_module_paths));
   *
   */
  void add_controller_module_paths(const std::vector<std::string> & paths);

  /*! \brief Add a controller to the enabled list
   *
   * This call enables a controller that was not enabled but loaded. If
   * the provided name does not exist then it does nothing and returns
   * false. Otherwise, the controller is created, added to the enabled map
   * and the function returns true.
   *
   * \param name Name of the controller to add
   */
  bool AddController(const std::string & name);

  /*! \brief Add an already created controller to the enabled list
   *
   * This call adds a controller that was created through another mean
   * than MCGlobalController internal mechanism.
   *
   * The function returns false and does nothing if name is already among
   * the enabled controllers or if controller is a null pointer.
   *
   * \param name Name of the controller to add
   *
   * \param controller Controller to add
   *
   */
  bool AddController(const std::string & name, std::shared_ptr<mc_control::MCController> controller);

  /*! \brief Switch to the requested controller
   *
   * If the requested controller is not enabled, does not exist or is already
   * running then this call has no effect. Otherwise, it will trigger a
   * controller switch at the next run call.
   *
   * \param name Name of the new controller to load
   */
  bool EnableController(const std::string & name);

  /** @name Grippers
   *
   * These functions can be used to manipulate the grippers through the global
   * controller interface
   *
   * @{
   */

  /*! \brief Get the current (active) joints values for the grippers */
  std::map<std::string, std::vector<double>> gripperQ();
  /*! \brief Get the joints in the grippers */
  std::map<std::string, std::vector<std::string>> gripperJoints();
  /*! \brief Get the active joints in the grippers */
  std::map<std::string, std::vector<std::string>> gripperActiveJoints();
  /*! \brief Set the current values of (active) joints in the grippers */
  void setGripperCurrentQ(const std::map<std::string, std::vector<double>> & gripperQs);
  /*! \brief Set the opening target(s) for a given gripper
   * \param name Name of the gripper
   * \param q Active joints values
   */
  void setGripperTargetQ(const std::string & name, const std::vector<double> & q);
  /*! \brief Set the gripper opening percentage for all grippers
   * \param pOpen Opening percentage (0: closed, 1: open)
   */
  void setGripperOpenPercent(double pOpen);
  /*! \brief Set the gripper opening percentage for a given gripper
   * \param name Name of the gripper
   * \param pOpen Opening percentage (0: closed, 1: open)
   */
  void setGripperOpenPercent(const std::string & name, double pOpen);
  /** @} */

  /** @name Services
   *
   * These functions acts as a proxy between the caller and the current
   * controller.
   *
   * These services can be active or inactive for each individual controller.
   * You will find some information in this document that only pertains to the
   * default implementation of the service call.
   *
   * @{
   *
   */

  /*! \brief Set a given joint position
   *
   * The value of pos is not checked w.r.t the angular limits of the given
   * joint as this should be insured by the controller.
   *
   * \param name Name of the joint
   *
   * \param pos Position value
   *
   * \returns True if successful (the joint exists and has one dof), false
   * otherwise
   */
  bool set_joint_pos(const std::string & jname, const double & pos);

  /*! \brief Get a given joint position
   *
   * \param name Name of the joint
   *
   * \param pos Will retrieve the given joint value
   *
   * \returns True if successful
   */
  bool get_joint_pos(const std::string & jname, double & pos);

  /*! \brief Change the end-effector controlled by the service
   *
   * \param ef_name Name of the end-effector
   *
   * \returns True if successful (the body exists within the robot), false
   * otherwise
   *
   */
  bool change_ef(const std::string & ef_name);

  /*! \brief Translate the current end-effector by a given amount
   *
   * \param t Translation to apply (in world frame)
   *
   * \returns True if successful
   *
   */
  bool translate_ef(const Eigen::Vector3d & t);

  /*! \brief Rotate the current end-effector by a given amount
   *
   * \param m Rotation matrix to apply (in world frame)
   *
   * \returns True if successful
   */
  bool rotate_ef(const Eigen::Matrix3d & m);

  /*! \brief Move the main robot's CoM by a given amount
   *
   * \param v Translation to apply to the CoM (in world frame)
   *
   * \returns True is succesful
   *
   */
  bool move_com(const Eigen::Vector3d & v);

  /*! \brief Trigger the next step in an FSM
   *
   * \returns True if successful
   */
  bool play_next_stance();

  /*! \brief Returns to half-sit pose after an experiment
   *
   * This service enables the HalfSitPose controller which only drives the
   * robot back to half-sitting posture while avoiding auto-collisions
   *
   */
  bool GoToHalfSitPose_service();
  /*! \brief See mc_rtc::MCGlobalController::GoToHalfSitPose_service */
  bool GoToHalfSitPose();

  /*! \brief Used to control driving controller
   *
   * \param w Wheel angle
   *
   * \param a Ankle angle
   *
   * \param p Pan angle
   *
   * \param t Tilt angle
   *
   * \returns True if successful
   *
   */
  bool driving_service(double w, double a, double p, double t);

  /*! \brief Send a message to the controller instance
   *
   * This method should be the preferred way to pass message to the controller
   *
   * \param msg Message passed to the controller
   *
   * \returns True if the controller successfully handled the message
   *
   */
  bool send_msg(const std::string & msg);

  /*! \brief Send a message and receive an answer from the controller
   *
   * This method should be the preferred way to pass/receive message to/from
   * the controller
   *
   * \param msg Message passed to the controller
   *
   * \param out Message received from the controller
   *
   * \returns True if the controller successfully handled the message
   *
   */
  bool send_recv_msg(const std::string & msg, std::string & out);

  /** @} */
public:
  /*! \brief Returns true if the controller is running
   *
   * To prevent any serious issues, the controller will stop when the
   * underlying MCController instance fails to run
   *
   */
  bool running;
private:
  /*! \brief Store the controller configuration */
  struct GlobalConfiguration
  {
    GlobalConfiguration(const std::string & conf,
                        std::shared_ptr<mc_rbdyn::RobotModule> rm);

    inline bool enabled(const std::string & ctrl);

    bool use_sandbox = false;

    bool verbose_loader = true;

    std::vector<std::string> robot_module_paths = {};
    std::shared_ptr<mc_rbdyn::RobotModule> main_robot_module;

    std::vector<std::string> controller_module_paths = {};
    std::vector<std::string> enabled_controllers = {};
    std::string initial_controller = "";
    double timestep = 0.002;

    bool update_real = true;
    bool update_real_from_sensors = false;

    bool publish_control_state = true;
    bool publish_env_state = true;
    bool publish_real_state = true;
    double publish_timestep = 0.01;

    bool enable_log = true;
    mc_rtc::Logger::Policy log_policy = mc_rtc::Logger::Policy::NON_THREADED;
    bfs::path log_directory;
    std::string log_template = "mc-control";

    Configuration config;
  };
private:
  GlobalConfiguration config;
  std::string current_ctrl = "";
  std::string next_ctrl = "";
  MCController * controller_ = nullptr;
  MCController * next_controller_ = nullptr;
  std::unique_ptr<mc_rtc::ObjectLoader<MCController>> controller_loader;
  std::map<std::string, std::shared_ptr<mc_control::MCController>> controllers;

  std::shared_ptr<mc_rbdyn::Robots> real_robots = nullptr;

  void publish_robots();

  void start_log();
  void setup_log();
  std::map<std::string, bool> setup_logger_ = {};
};

}
