/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_filter/ExponentialMovingAverage.h>
#include <mc_filter/LeakyIntegrator.h>
#include <mc_filter/StationaryOffset.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/OrientationTask.h>

#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>

#include <Eigen/QR>
#include <eigen-quadprog/QuadProg.h>

namespace mc_tasks
{

namespace lipm_stabilizer
{

/** Walking stabilization based on linear inverted pendulum tracking.
 *
 * Stabilization bridges the gap between the open-loop behavior of the
 * pendulum state reference (feedforward controls) and feedback read from
 * state estimation. In our case, feedback is done on the DCM of the LIPM:
 *
 * \f[
 *   \dot{\xi} = \dot{\xi}^{d} + k_p (\xi^d - \xi) + k_i \int (\xi^d - \xi)
 * \f]
 *
 * Which boils down into corresponding formulas for the CoP and CoM
 * acceleration targets.
 */
struct MC_TASKS_DLLAPI StabilizerTask : public MetaTask
{

  static constexpr double MAX_AVERAGE_DCM_ERROR = 0.05; /**< Maximum average (integral) DCM error in [m] */
  static constexpr double MAX_COM_ADMITTANCE = 20; /**< Maximum admittance for CoM admittance control */
  static constexpr double MAX_COP_ADMITTANCE = 0.1; /**< Maximum CoP admittance for foot damping control */
  static constexpr double MAX_DCM_D_GAIN = 2.; /**< Maximum DCM derivative gain (no unit) */
  static constexpr double MAX_DCM_I_GAIN = 100.; /**< Maximum DCM average integral gain in [Hz] */
  static constexpr double MAX_DCM_P_GAIN = 20.; /**< Maximum DCM proportional gain in [Hz] */
  static constexpr double MAX_DFZ_ADMITTANCE =
      5e-4; /**< Maximum admittance in [s] / [kg] for foot force difference control */
  static constexpr double MAX_DFZ_DAMPING =
      10.; /**< Maximum normalized damping in [Hz] for foot force difference control */
  static constexpr double MAX_FDC_RX_VEL =
      0.2; /**< Maximum x-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_FDC_RY_VEL =
      0.2; /**< Maximum y-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MAX_FDC_RZ_VEL =
      0.2; /**< Maximum z-axis angular velocity in [rad] / [s] for foot damping control. */
  static constexpr double MIN_DS_PRESSURE = 15.; /**< Minimum normal contact force in DSP, used to avoid low-pressure
                                                    targets when close to contact switches. */
  /**< Minimum force for valid ZMP computation (throws otherwise) */
  static constexpr double MIN_NET_TOTAL_FORCE_ZMP = 1.;

public:
  /**
   * @brief Creates a stabilizer meta task
   *
   * @param robots Robots on which the task acts
   * @param realRobots Corresponding real robot instances
   * @param robotIndex Index of the robot to stabilize
   * @param leftSurface Left foot surface name. Its origin should be the center of the foot sole
   * @param rightSurface Left foot surface name. Its origin should be the center of the foot sole
   * @param torsoBodyName Body name of the robot's torso (i.e a link above the
   * floating base)
   * @param dt Controller's timestep
   */
  StabilizerTask(const mc_rbdyn::Robots & robots,
                 const mc_rbdyn::Robots & realRobots,
                 unsigned int robotIndex,
                 const std::string & leftSurface,
                 const std::string & rightSurface,
                 const std::string & torsoBodyName,
                 double dt);

  /**
   * @brief Resets the stabilizer tasks and parameters to their default configuration.
   *
   * Resets all tasks and errors/integrator/derivators to their initial
   * configuration. Configures the default stabilizer parameters from the robot
   * module.
   *
   * You can configure the stabilizer parameters (DCM tacking gains, task gains, etc) by calling
   * configure(const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config)
   */
  void reset() override;

  /*! \brief Returns the task error
   *
   * Since the StabilizerTask is a MetaTask, the vector is a concatenation of each
   * sub-tasks. The vector's dimensions depend on the underlying task, and the
   * sub-tasks evaluation depends on their order of insertion.
   */
  Eigen::VectorXd eval() const override;

  /*! \brief Returns the task velocity
   *
   * Since the StabilizerTask is a MetaTask, the vector is a concatenation of each
   * sub-tasks. The vector's dimensions depend on the underlying task, and the
   * sub-tasks evaluation depends on their order of insertion.
   */
  Eigen::VectorXd speed() const override;

  /**
   * @brief Enables stabilizer
   *
   * This will reinitialize all integrators, and set the stabilizer gains
   * according to the last call to configure()
   */
  void enable();

  /** Disable all feedback components. */
  void disable();

  /** Configure stabilizer's parameters from a stabilizer's configuration object
   *
   * @param config Stabilizer configuration. Default values can be found in the
   * RobotModule, and modified from YAML configuration or manually.
   *
   * \see load(mc_solver::QPSolver &, const mc_rtc::Configuration &) to set
   * stabilizer targets and contacts from configuration
   */
  void configure(const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config);

  /*! \brief Load targets and contacts from configuration */
  void load(mc_solver::QPSolver &, const mc_rtc::Configuration & config) override;

  /**
   * @brief Get current stabilizer's configuration (including changes from GUI)
   */
  const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config() const;

  /**
   * Reset stabilizer configuration from last configuration set by configure()
   *
   * Does not include changes made from the GUI.
   */
  void reconfigure();

  /** Update QP task targets.
   *
   * This function is called once the reference has been updated.
   */
  void run();

  /** Configure foot tasks for contact at a given location, and add contacts to the solver.
   *
   * \note To use the stabilizer with dynamics constraint, you need to add the
   * corresponding mc_rbdyn::Contact to the solver and free the roll/pitch rotation and z translation (in contact
   * frame). This assumes the foot surfaces to have x pointing towards the front of the foot, and z from the ground up.
   *
   * \param solver The QP solver to which the contact tasks will be added. Note
   * that this method will not add contact constraints to the QPSolver. If you
   * wish to do so, bear in mind that for the stabilizer to work properly, the
   * contact's dofs along the x and y rotations and z translation need to be
   * free.
   */
  void setContacts(mc_solver::QPSolver & solver,
                   const std::vector<std::pair<ContactState, internal::Contact>> & contacts);

  /** Helper to set contacts from the current surface pose
   *
   * @param contacts Contacts to add. Their pose will be determined set from the
   * realRobot estimate of the foot surface pose. Use with caution.
   *
   * \see void setContacts(mc_solver::QPSolver &, const std::vector<std::pair<ContactState, sva::PTransformd>> &);
   */
  void setContacts(mc_solver::QPSolver & solver, const std::vector<ContactState> & contacts);

  /**
   * @brief Projected pose of the ankle frame in the contact frame.
   *
   * @param s Contact for which the frame is requested
   *
   * @return The projected ankle frame expressed in world frame.
   */
  const sva::PTransformd & contactAnklePose(ContactState s) const
  {
    return contacts_.at(s).anklePose();
  }

  const std::string & footSurface(ContactState s) const
  {
    return footTasks.at(s)->surface();
  }

  /**
   * @brief Interpolation paremeter between left and right foot
   *
   * @return Left foot ratio between [0,1]
   */
  double leftFootRatio() const
  {
    return leftFootRatio_;
  }

  /**
   * @brief computes the anchorFrame compatible with the state observers
   * (KinematicInertial)
   *
   * @return Anchor frame in-between the feet according to leftFootRatio()
   */
  sva::PTransformd anchorFrame() const;
  /**
   * @brief Returns the anchor frame computed from real robot
   */
  sva::PTransformd anchorFrameReal() const;

  /** ZMP target after force distribution.
   *
   */
  Eigen::Vector3d zmp() const
  {
    return distribZMP_;
  }

  /** Provides a static target to the stabilizer.
   * - CoM target : user-provided
   * - CoM velocity target: zero (static)
   * - CoM acceleration target: zero (static)
   * - ZMP: computed under the CoM
   *
   * @param com desired com position
   *
   * @param zmpHeight[=0] optional height of the ZMP plane (eg contacts), used to compute the
   * pendulums omega.
   *
   * \see target for dynamic motions.
   */
  void staticTarget(const Eigen::Vector3d & com, double zmpHeight = 0);

  /**
   * @brief Provides a dynamic target to the stabilizer.
   *
   * Note that this target should be updated at each iteration and provide a
   * dynamically-consistent trajectory. This would typically be generated by a
   * compatible Model Preview Controller.
   *
   * See https://github.com/jrl-umi3218/lipm_walking_controller for example in the context of walking.
   *
   * @param com Desired CoM position
   * @param comd Desired CoM velocity
   * @param comdd Desired CoM acceleration
   * @param zmp Desired ZMP
   *
   * \see staticTarget for a helper to define the stabilizer target when the CoM
   * is static
   */
  void target(const Eigen::Vector3d & com,
              const Eigen::Vector3d & comd,
              const Eigen::Vector3d & comdd,
              const Eigen::Vector3d & zmp);

  const Eigen::Vector3d & measuredDCM()
  {
    return measuredDCM_;
  }

  const Eigen::Vector3d & measuredZMP()
  {
    return measuredZMP_;
  }

  const Eigen::Vector3d & measuredCoM()
  {
    return measuredCoM_;
  }

  const Eigen::Vector3d & measuredCoMd()
  {
    return measuredCoM_;
  }

  bool inContact(ContactState state) const
  {
    return contacts_.count(state);
  }

  bool inDoubleSupport() const
  {
    return contacts_.size() == 2;
  }

private:
  void dimWeight(const Eigen::VectorXd & dimW) override;
  Eigen::VectorXd dimWeight() const override;

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  virtual void selectUnactiveJoints(
      mc_solver::QPSolver & solver,
      const std::vector<std::string> & unactiveJointsName,
      const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  /**
   * @brief Add contact to the task (set tasks target, update support area).
   * This function does not immediately add contact tasks to the solver, this
   * will be done by update() when the task is added to the solver.
   */
  void addContact(ContactState contactState, const internal::Contact & contact);

  /** Check that all gains are within boundaries. */
  void checkGains();

  /** Check whether the robot is in the air. */
  void checkInTheAir();

  /** Computes the ratio of force distribution between the feet based on
   * the reference CoM and contact ankle positions.
   */
  void computeLeftFootRatio();

  /** Update real-robot state.
   *
   * \param com Position of the center of mass.
   *
   * \param comd Velocity of the center of mass.
   */
  void updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd);

  /**
   * @brief Update contact tasks in the solver
   *
   * @param solver QPSolver holding the tasks
   */
  void updateContacts(mc_solver::QPSolver & solver);

  /** Compute desired wrench based on DCM error.
   *
   */
  sva::ForceVecd computeDesiredWrench();

  /** Distribute a desired wrench in double support.
   *
   * \param desiredWrench Desired resultant reaction wrench.
   */
  void distributeWrench(const sva::ForceVecd & desiredWrench);

  /** Project desired wrench to single support foot.
   *
   * \param desiredWrench Desired resultant reaction wrench.
   *
   * \param footTask Target foot.
   *
   * \param target contact
   */
  void saturateWrench(const sva::ForceVecd & desiredWrench,
                      std::shared_ptr<mc_tasks::force::CoPTask> & footTask,
                      const internal::Contact & contact);

  /** Reset admittance, damping and stiffness for every foot in contact. */
  void setSupportFootGains();

  /** Apply foot force difference control.
   *
   * This method is described in Section III.E of "Biped walking
   * stabilization based on linear inverted pendulum tracking" (Kajita et
   * al., IROS 2010).
   */
  void updateFootForceDifferenceControl();

  /** Update ZMP frame from contact state. */
  void updateZMPFrame();

  /** Get 6D contact admittance vector from 2D CoP admittance. */
  sva::ForceVecd contactAdmittance() const
  {
    return {{c_.copAdmittance.y(), c_.copAdmittance.x(), 0.}, {0., 0., 0.}};
  }

  /* Task-related properties */
protected:
  void addToSolver(mc_solver::QPSolver & solver) override;
  void removeFromSolver(mc_solver::QPSolver & solver) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;
  void update(mc_solver::QPSolver &) override;

  /** Log stabilizer entries.
   *
   * \param logger Logger.
   */
  void addToLogger(mc_rtc::Logger &) override;
  void removeFromLogger(mc_rtc::Logger &) override;
  void addToGUI(mc_rtc::gui::StateBuilder &) override;

  const mc_rbdyn::Robot & robot() const
  {
    return robots_.robot(robotIndex_);
  }

  const mc_rbdyn::Robot & realRobot() const
  {
    return realRobots_.robot(robotIndex_);
  }

protected:
  /**
   * @brief Workaround a C++11 standard bug: no specialization of the hash
   * functor exists for enum types.
   * Fixed in GCC 6.1 and clang's libc++ in 2013
   *
   * See http://www.open-std.org/jtc1/sc22/wg21/docs/lwg-defects.html#2148
   */
  struct EnumClassHash
  {
    template<typename T>
    std::size_t operator()(T t) const
    {
      return static_cast<std::size_t>(t);
    }
  };
  std::unordered_map<ContactState, internal::Contact, EnumClassHash> contacts_;
  std::vector<ContactState> addContacts_; /**< Contacts to add to the QPSolver when the task is inserted */
  std::unordered_map<ContactState, std::shared_ptr<mc_tasks::force::CoPTask>, EnumClassHash> footTasks;
  std::vector<std::shared_ptr<mc_tasks::force::CoPTask>> contactTasks;

  std::vector<std::vector<Eigen::Vector3d>> supportPolygons_; /**< For GUI display */
  Eigen::Vector2d supportMin_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d supportMax_ = Eigen::Vector2d::Zero();
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::OrientationTask> pelvisTask; /**< Pelvis orientation task */
  std::shared_ptr<mc_tasks::OrientationTask> torsoTask; /**< Torso orientation task */
  const mc_rbdyn::Robots & robots_;
  const mc_rbdyn::Robots & realRobots_;
  unsigned int robotIndex_;

  /** Stabilizer targets */
  Eigen::Vector3d comTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comdTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comddTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmTarget_ = Eigen::Vector3d::Zero();
  double omega_;

  double t_ = 0.; /**< Time elapsed since the task is running */

protected:
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration
      defaultConfig_; /**< Default (user-provided) configuration for the stabilizer. This configuration is superseeded
                         by the parameters set in the GUI */
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration
      c_; /* Online stabilizer configuration, can be set from the GUI. Defaults to defaultConfig_ */
  Eigen::QuadProgDense qpSolver_; /**< Least-squares solver for wrench distribution */
  Eigen::Vector3d dcmAverageError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmVelError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoM_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoMd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredZMP_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredDCM_ = Eigen::Vector3d::Zero();
  sva::ForceVecd measuredNetWrench_;
  Eigen::Vector3d zmpError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d distribZMP_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d polePlacement_ = {-10., -5., -1., 10.}; /**< Pole placement with ZMP delay (Morisawa et al., 2014) */
  mc_filter::ExponentialMovingAverage<Eigen::Vector3d> dcmIntegrator_;
  mc_filter::StationaryOffset<Eigen::Vector3d> dcmDerivator_;
  bool inTheAir_ = false; /**< Is the robot in the air? */
  double dfzForceError_ = 0.; /**< Force error in foot force difference control */
  double dfzHeightError_ = 0.; /**< Height error in foot force difference control */
  double dt_ = 0.005; /**< Controller cycle in [s] */
  double leftFootRatio_ = 0.5; /**< Weight distribution ratio (0: all weight on right foot, 1: all on left foot) */
  double mass_ = 38.; /**< Robot mass in [kg] */
  double runTime_ = 0.;
  double vdcHeightError_ = 0.; /**< Average height error used in vertical drift compensation */
  sva::ForceVecd distribWrench_ = sva::ForceVecd::Zero();
  std::vector<std::string> sensorNames_ = {
      "LeftFootForceSensor", "RightFootForceSensor"}; /** Force sensors corresponding to established contacts */
  sva::PTransformd zmpFrame_;
};

} // namespace lipm_stabilizer
} // namespace mc_tasks
