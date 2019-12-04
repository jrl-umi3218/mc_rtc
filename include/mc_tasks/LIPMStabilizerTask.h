/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_rbdyn/lipm_stabilizer/Contact.h>
#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>
#include <mc_signal/ExponentialMovingAverage.h>
#include <mc_signal/LeakyIntegrator.h>
#include <mc_signal/StationaryOffsetFilter.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/OrientationTask.h>

#include <Eigen/QR>
#include <eigen-quadprog/QuadProg.h>

namespace mc_tasks
{

namespace lipm_stabilizer
{

using ContactState = mc_rbdyn::lipm_stabilizer::ContactState;
using Contact = mc_rbdyn::lipm_stabilizer::Contact;
using Sole = mc_rbdyn::lipm_stabilizer::Sole;

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
  static constexpr double MAX_ZMPCC_COM_OFFSET = 0.05; /**< Maximum CoM offset due to admittance control in [m] */
  static constexpr double MIN_DS_PRESSURE = 15.; /**< Minimum normal contact force in DSP, used to avoid low-pressure
                                                    targets when close to contact switches. */
  /**< Minimum force for valid ZMP computation (throws otherwise) */
  static constexpr double MIN_NET_TOTAL_FORCE_ZMP = 1.;

  /**< Gravity (ISO 80000-3) */
  static constexpr double GRAVITY = 9.80665;

public:
  StabilizerTask(const mc_rbdyn::Robots & robots,
                 const mc_rbdyn::Robots & realRobots,
                 unsigned int robotIndex,
                 const std::string & leftSurface,
                 const std::string & rightSurface,
                 const std::string & torsoBodyName,
                 double dt);
  ~StabilizerTask() override;

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

  /*! \brief Returns the task error
   *
   * Since the StabilizerTask is a MetaTask, the vector is a concatenation of each
   * sub-tasks. The vector's dimensions depend on the underlying task.
   *
   * [CoM eval = CoMTask::eval(), Left foot eval = CoPTask::eval(), Right foot eval = CoPTask::eval()]
   */
  Eigen::VectorXd eval() const override;

  /*! \brief Returns the task velocity
   *
   * Since the StabilizerTask is a MetaTask, the vector is a concatenation of each
   * sub-tasks. The vector's dimensions depend on the underlying task.
   *
   * [CoM speed = CoMTask::speed(), Left foot speed = CoPTask::speed(), Right foot speed = CoPTask::speed()]
   */
  Eigen::VectorXd speed() const override;

  /** Stabilizer specific */
public:
  /** Add GUI panel.
   *
   * \param gui GUI handle.
   *
   */
  void addGUIElements(std::shared_ptr<mc_rtc::gui::StateBuilder> gui);

  /**
   * @brief Enables stabilizer
   *
   * This will reinitialize all integrators, and set the stabilizer gains
   * according to the last call to configure()
   */
  void enable();

  /** Disable all feedback components.
   *
   */
  void disable();

  /** Setup stabilizer configuration
   *
   * @param config Stabilizer configuration. Default values can be found in the
   * RobotModule, and modified from YAML configuration or manually.
   */
  void configure(const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config);

  /**
   * @brief Get current stabilizer's configuration
   *
   * @return Stabilizer configuration
   */
  const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config() const;

  /**
   * Reset stabilizer configuration from last valid default configuration set by configure()
   */
  void reconfigure();

  /** Detect foot touchdown based on both force and distance.
   *
   * \param footTask Swing foot task.
   *
   * \param contact Target contact.
   *
   */
  bool detectTouchdown(const std::shared_ptr<mc_tasks::force::CoPTask> footTask, const Contact & contact);

  /** Update QP task targets.
   *
   * This function is called once the reference has been updated.
   *
   */
  void run();

  /** Configure foot task for contact seeking.
   *
   * \param footTask One of leftFootTask or rightFootTask.
   *
   * This function has no effect when the measured pressure is already higher
   * than the target. Otherwise, it will set a positive admittance along the
   * z-axis of the contact frame.
   *
   */
  void seekTouchdown(std::shared_ptr<mc_tasks::force::CoPTask> footTask);

  /** Configure foot tasks for contact at a given location, and add contacts to
   * the solver.
   *
   * - For all feet in contact, the current control foot position will be used as the contact frame
   * - When in ContactState::LeftFoot or ContactState::RightFoot, the free foot is configured as a position task
   * allowing to track a trajectory
   *
   * \note To use the stabilizer with dynamics constraint, you need to add the
   * corresponding mc_rbdyn::Contact to the solver and free the roll/pitch rotation and z translation (in contact
   * frame). This assumes the foot surfaces to have x pointing towards the front of the foot, and z from the ground up.
   *
   */
  void setContacts(ContactState state);

  /** Get contact state.
   *
   */
  ContactState contactState() const
  {
    return contactState_;
  }

  /** Update real-robot state.
   *
   * \param com Position of the center of mass.
   *
   * \param comd Velocity of the center of mass.
   *
   * \param leftFootRatio Desired pressure distribution ratio for left foot.
   *
   */
  void updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, double leftFootRatio);

  /**
   * @brief Access the left foot ratio parameter
   *
   * @return Left foot ratio
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
  sva::PTransformd anchorFrame() const
  {
    return sva::interpolate(leftFootTask->surfacePose(), rightFootTask->surfacePose(), leftFootRatio_);
  }

  /** Update H-representation of contact wrench cones.
   *
   * \param sole Sole parameters.
   *
   * See <https://hal.archives-ouvertes.fr/hal-02108449/document> for
   * technical details on the derivation of this formula.
   *
   */
  void wrenchFaceMatrix(const Sole & sole)
  {
    double X = sole.halfLength;
    double Y = sole.halfWidth;
    double mu = sole.friction;
    // clang-format off
    wrenchFaceMatrix_ <<
      // mx,  my,  mz,  fx,  fy,            fz,
          0,   0,   0,  -1,   0,           -mu,
          0,   0,   0,  +1,   0,           -mu,
          0,   0,   0,   0,  -1,           -mu,
          0,   0,   0,   0,  +1,           -mu,
         -1,   0,   0,   0,   0,            -Y,
         +1,   0,   0,   0,   0,            -Y,
          0,  -1,   0,   0,   0,            -X,
          0,  +1,   0,   0,   0,            -X,
        +mu, +mu,  -1,  -Y,  -X, -(X + Y) * mu,
        +mu, -mu,  -1,  -Y,  +X, -(X + Y) * mu,
        -mu, +mu,  -1,  +Y,  -X, -(X + Y) * mu,
        -mu, -mu,  -1,  +Y,  +X, -(X + Y) * mu,
        +mu, +mu,  +1,  +Y,  +X, -(X + Y) * mu,
        +mu, -mu,  +1,  +Y,  -X, -(X + Y) * mu,
        -mu, +mu,  +1,  -Y,  +X, -(X + Y) * mu,
        -mu, -mu,  +1,  -Y,  -X, -(X + Y) * mu;
    // clang-format on
  }

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
   * \see dynamicTarget for dynamic motions.
   */
  void staticTarget(const Eigen::Vector3d & com);

  // void dynamicTarget(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, const Eigen::Vector3d & comdd, const
  // Eigen::Vector3d & zmp);

private:
  /** Check that all gains are within boundaries.
   *
   */
  void checkGains();

  /** Check whether the robot is in the air.
   *
   */
  void checkInTheAir();

  /** Compute desired wrench based on DCM error.
   *
   */
  sva::ForceVecd computeDesiredWrench();

  /** Compute ZMP of a wrench in the output frame.
   *
   * \param wrench Wrench at the origin of the world frame.
   *
   */
  Eigen::Vector3d computeZMP(const sva::ForceVecd & wrench) const;

  /** Distribute a desired wrench in double support.
   *
   * \param desiredWrench Desired resultant reaction wrench.
   *
   */
  void distributeWrench(const sva::ForceVecd & desiredWrench);

  /** Project desired wrench to single support foot.
   *
   * \param desiredWrench Desired resultant reaction wrench.
   *
   * \param footTask Target foot.
   *
   */
  void saturateWrench(const sva::ForceVecd & desiredWrench, std::shared_ptr<mc_tasks::force::CoPTask> & footTask);

  /** Reset admittance, damping and stiffness for every foot in contact.
   *
   */
  void setSupportFootGains();

  /** Update CoM task with ZMP Compensation Control.
   *
   * This approach is based on Section 6.2.2 of Dr Nagasaka's PhD thesis
   * "体幹位置コンプライアンス制御によるモデル誤差吸収" (1999) from
   * <https://sites.google.com/site/humanoidchannel/home/publication>.
   * The main differences is that the CoM offset is (1) implemented as CoM
   * damping control with an internal leaky integrator and (2) computed from
   * the distributed rather than reference ZMP.
   *
   */
  void updateCoMTaskZMPCC();

  /** Apply foot force difference control.
   *
   * This method is described in Section III.E of "Biped walking
   * stabilization based on linear inverted pendulum tracking" (Kajita et
   * al., IROS 2010).
   *
   */
  void updateFootForceDifferenceControl();

  /** Update ZMP frame from contact state.
   *
   */
  void updateZMPFrame();

  /** Get 6D contact admittance vector from 2D CoP admittance.
   *
   */
  sva::ForceVecd contactAdmittance()
  {
    return {{c_.copAdmittance.y(), c_.copAdmittance.x(), 0.}, {0., 0., 0.}};
  }

  /* Task-related properties */
protected:
  void addToSolver(mc_solver::QPSolver & solver) override;
  void removeFromSolver(mc_solver::QPSolver & solver) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;
  void update() override;

  /** Log stabilizer entries.
   *
   * \param logger Logger.
   *
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
  Contact leftFootContact_;
  Contact rightFootContact_;
  std::vector<std::vector<Eigen::Vector3d>> supportPolygons_; /**< For GUI display */
  Eigen::Vector3d supportMin_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d supportMax_ = Eigen::Vector3d::Zero();
  std::shared_ptr<mc_tasks::CoMTask> comTask;
  std::shared_ptr<mc_tasks::force::CoPTask> leftFootTask;
  std::shared_ptr<mc_tasks::force::CoPTask> rightFootTask;
  std::shared_ptr<mc_tasks::OrientationTask> pelvisTask; /**< Pelvis orientation task */
  std::shared_ptr<mc_tasks::OrientationTask> torsoTask; /**< Torso orientation task */
  const mc_rbdyn::Robots & robots_;
  const mc_rbdyn::Robots & realRobots_;
  unsigned int robotIndex_;
  std::string leftFootSurface_ = "LeftFootCenter";
  std::string rightFootSurface_ = "RightFootCenter";

  /** Stabilizer targets */
  Eigen::Vector3d comTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comdTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comddTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmTarget_ = Eigen::Vector3d::Zero();
  double omega_;

  double t_ = 0.; /**< Time elapsed since the task is running */

protected:
  Eigen::Vector3d gravity_ = {0., 0., -GRAVITY}; // ISO 80000-3}; /**< Gravity vector */
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration
      defaultConfig_; /**< Default (user-provided) configuration for the stabilizer. This configuration is superseeded
                         by the parameters set in the GUI */
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration
      c_; /* Online stabilizer configuration, can be set from the GUI. Defaults to defaultConfig_ */
  ContactState contactState_ = ContactState::DoubleSupport;
  Eigen::QuadProgDense qpSolver_; /**< Least-squares solver for wrench distribution */
  Eigen::Matrix<double, 16, 6> wrenchFaceMatrix_; /**< Matrix of single-contact wrench cone inequalities */
  Eigen::Vector3d dcmAverageError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmVelError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoM_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoMd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredZMP_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredDCM_ = Eigen::Vector3d::Zero();
  sva::ForceVecd measuredNetWrench_;
  Eigen::Vector3d zmpError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpccCoMAccel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpccCoMOffset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpccCoMVel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpccError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d distribZMP_ = Eigen::Vector3d::Zero();
  Eigen::Vector4d polePlacement_ = {-10., -5., -1., 10.}; /**< Pole placement with ZMP delay (Morisawa et al., 2014) */
  mc_signal::ExponentialMovingAverage<Eigen::Vector3d> dcmIntegrator_;
  mc_signal::LeakyIntegrator<Eigen::Vector3d> zmpccIntegrator_;
  mc_signal::StationaryOffsetFilter<Eigen::Vector3d> dcmDerivator_;
  bool inTheAir_ = false; /**< Is the robot in the air? */
  bool zmpccOnlyDS_ = true; /**< Apply CoM admittance control only in double support? */
  double dfzForceError_ = 0.; /**< Force error in foot force difference control */
  double dfzHeightError_ = 0.; /**< Height error in foot force difference control */
  double dt_ = 0.005; /**< Controller cycle in [s] */
  double leftFootRatio_ = 0.5; /**< Weight distribution ratio (0: all weight on right foot, 1: all on left foot) */
  double mass_ = 38.; /**< Robot mass in [kg] */
  double runTime_ = 0.;
  double vdcHeightError_ = 0.; /**< Average height error used in vertical drift compensation */
  sva::ForceVecd distribWrench_ = sva::ForceVecd::Zero();
  // XXX removed wrt to Stephane's version. Was only used to compute the ZMP, we
  // use Robot::zmp instead
  // sva::ForceVecd measuredWrench_; /**< Net contact wrench measured from sensors */

  // XXX should be the sensors in contact only (same issue in stephane's code),
  // otherwise while walking we're sensible to swing foot sensor noise
  std::vector<std::string> sensorNames_ = {"LeftFootForceSensor", "RightFootForceSensor"};
  sva::PTransformd zmpFrame_;
};

} // namespace lipm_stabilizer
} // namespace mc_tasks
