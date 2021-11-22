/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_filter/ExponentialMovingAverage.h>
#include <mc_filter/LeakyIntegrator.h>
#include <mc_filter/LowPass.h>
#include <mc_filter/StationaryOffset.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/OrientationTask.h>

#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>
#include <mc_tasks/lipm_stabilizer/ZMPCC.h>

#include <state-observation/dynamics-estimators/lipm-dcm-estimator.hpp>

#include <Eigen/QR>
#include <eigen-quadprog/QuadProg.h>

namespace mc_tasks
{

namespace lipm_stabilizer
{

using ::mc_filter::utils::clamp;
using ZMPCCConfiguration = mc_rbdyn::lipm_stabilizer::ZMPCCConfiguration;
using StabilizerConfiguration = mc_rbdyn::lipm_stabilizer::StabilizerConfiguration;
using FDQPWeights = mc_rbdyn::lipm_stabilizer::FDQPWeights;
using SafetyThresholds = mc_rbdyn::lipm_stabilizer::SafetyThresholds;
using DCMBiasEstimatorConfiguration = mc_rbdyn::lipm_stabilizer::DCMBiasEstimatorConfiguration;
using ExternalWrenchConfiguration = mc_rbdyn::lipm_stabilizer::ExternalWrenchConfiguration;

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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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
   * @brief Creates a stabilizer meta task
   *
   * This constructor uses the stabilizer configuration in the robot module associated to the controlled robot. The
   * stabilizer is started with two feet contacts.
   *
   * @param robots Robots on which the task acts
   *
   * @param realRobots Corresponding real robots instance
   *
   * @param robotIndex Index of the robot
   *
   * @param dt Controller's timestep
   */
  StabilizerTask(const mc_rbdyn::Robots & robots,
                 const mc_rbdyn::Robots & realRobots,
                 unsigned int robotIndex,
                 double dt);

  inline void name(const std::string & name) override
  {
    name_ = name;

    // Rename the tasks managed by the stabilizer
    // Doing so helps making the logs more consistent, and having a fixed name
    // allows for predifined custom plots in the log ui.
    const auto n = name_ + "_Tasks";
    comTask->name(n + "_com");
    footTasks.at(ContactState::Left)->name(n + "_cop_left");
    footTasks.at(ContactState::Right)->name(n + "_cop_right");
    pelvisTask->name(n + "_pelvis");
    torsoTask->name(n + "_torso");
  }

  using MetaTask::name;

  /**
   * @brief Resets the stabilizer tasks and parameters to their default configuration.
   *
   * Resets all tasks and errors/integrator/derivators to their initial
   * configuration. Configures the default stabilizer parameters from the robot
   * module.
   *
   * You can configure the stabilizer parameters (DCM tacking gains, task gains, etc) by calling
   * configure(const StabilizerConfiguration & config)
   *
   * \note If you wish to reset the stabilizer from it's current configuration,
   * you can do so by storing its current configuration as accessed by config()
   * or commitedConfig() and set it explitely after calling reset by calling
   * configure(const StabilizerConfiguration &);
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
  void configure(const StabilizerConfiguration & config);

  /**
   * @brief Use the current configuration as the new default
   */
  void commitConfig();

  /*! \brief Load targets and contacts from configuration */
  void load(mc_solver::QPSolver &, const mc_rtc::Configuration & config) override;

  /**
   * @brief Get current stabilizer's configuration (including changes from
   * GUI/code)
   *
   * \see commitedConfig()
   */
  const StabilizerConfiguration & config() const;

  /**
   * @brief Get last commited configuration
   *
   * Commited configuration is corresponds to the latest one set by calling commitConfig(), either manually or from the
   * GUI.
   *
   * \see config()
   */
  const StabilizerConfiguration & commitedConfig() const;

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
  void setContacts(const ContactDescriptionVector & contacts);

  /**
   * @brief Helper to set contacts with a provided target pose
   *
   * @param contacts vectors of contacts defined by their ContactState and a
   * desired pose
   */
  void setContacts(const std::vector<std::pair<ContactState, sva::PTransformd>> & contacts);

  /** Helper to set contacts from the current surface pose
   *
   * @param contacts Contacts to add. Their pose will be determined set from the
   * realRobot estimate of the foot surface pose. Use with caution.
   *
   * \see void setContacts(mc_solver::QPSolver &, const std::vector<std::pair<ContactState, sva::PTransformd>> &);
   */
  void setContacts(const std::vector<ContactState> & contacts);

  /**
   * @brief Projected pose of the ankle frame in the contact frame.
   *
   * @param s Contact for which the frame is requested
   *
   * @return The projected ankle frame expressed in world frame.
   */
  inline const sva::PTransformd & contactAnklePose(ContactState s) const
  {
    return contacts_.at(s).anklePose();
  }

  inline const std::string & footSurface(ContactState s) const
  {
    return footTasks.at(s)->surface();
  }

  /**
   * @brief Interpolation paremeter between left and right foot
   *
   * @return Left foot ratio between [0,1]
   */
  inline double leftFootRatio() const noexcept
  {
    return leftFootRatio_;
  }

  /**
   * @brief computes the anchorFrame compatible with the state observers
   * (e.g KinematicInertial)
   *
   * @param robot Robot from which the frame will be computed
   *
   * @return Anchor frame in-between the feet according to leftFootRatio()
   */
  sva::PTransformd anchorFrame(const mc_rbdyn::Robot & robot) const;

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
   * @param zmpd Desired ZMP velocity (can be omitted when zmpdGain in StabilizerConfiguration is zero)
   *
   * \see staticTarget for a helper to define the stabilizer target when the CoM
   * is static
   */
  void target(const Eigen::Vector3d & com,
              const Eigen::Vector3d & comd,
              const Eigen::Vector3d & comdd,
              const Eigen::Vector3d & zmp,
              const Eigen::Vector3d & zmpd = Eigen::Vector3d::Zero());

  /**
   * @brief Set the wrench that the robot expects to receive from the external contacts.
   *
   * Change the configurations in ExternalWrenchConfiguration to handle external wrenches because external wrenches are
   * ignored by default. \see ExternalWrenchConfiguration
   *
   * @param surfaceNames Names of the surface to which the external wrench is applied
   * @param targetWrenches Target (expected) external wrenches
   * @param gains Gains of measured external wrenches
   */
  void setExternalWrenches(const std::vector<std::string> & surfaceNames,
                           const std::vector<sva::ForceVecd> & targetWrenches,
                           const std::vector<sva::MotionVecd> & gains);

  inline const Eigen::Vector3d & measuredDCM() noexcept
  {
    return measuredDCM_;
  }

  inline const Eigen::Vector3d & measuredZMP() noexcept
  {
    return measuredZMP_;
  }

  inline const Eigen::Vector3d & measuredCoM() noexcept
  {
    return measuredCoM_;
  }

  inline const Eigen::Vector3d & measuredCoMd() noexcept
  {
    return measuredCoM_;
  }

  inline const Eigen::Vector3d & comOffsetTarget() noexcept
  {
    return comOffsetTarget_;
  }

  inline bool inContact(ContactState state) const noexcept
  {
    return contacts_.count(state);
  }

  inline bool inDoubleSupport() const noexcept
  {
    return contacts_.size() == 2;
  }

  inline const mc_rbdyn::Robot & robot() const noexcept
  {
    return robots_.robot(robotIndex_);
  }

  inline const mc_rbdyn::Robot & realRobot() const noexcept
  {
    return realRobots_.robot(robotIndex_);
  }

  /**
   * @name Setters to reconfigure the stabilizer online
   *
   * Setters for the main parameters of the stabilizer.
   * For safety purposes, values are clamped against the maximum values defined
   * by the stabilizer configuration.
   *
   * \see StabilizerConfiguration for details on each
   * of these parameters.
   *
   * \see config() Access the current stabilizer configuration values
   * \see commitConfig() Make the current configuration the new default
   * @{
   */
  inline void torsoPitch(double pitch) noexcept
  {
    c_.torsoPitch = pitch;
  }

  inline void torsoWeight(double weight) noexcept
  {
    c_.torsoWeight = weight;
    torsoTask->weight(c_.torsoWeight);
  }

  inline void torsoStiffness(double stiffness) noexcept
  {
    c_.torsoStiffness = stiffness;
    torsoTask->stiffness(stiffness);
  }

  inline void pelvisWeight(double weight) noexcept
  {
    c_.pelvisWeight = weight;
    pelvisTask->weight(c_.pelvisWeight);
  }

  inline void pelvisStiffness(double stiffness) noexcept
  {
    c_.pelvisStiffness = stiffness;
    pelvisTask->stiffness(stiffness);
  }

  inline void dcmGains(double p, double i, double d) noexcept
  {
    c_.dcmPropGain = clamp(p, 0., c_.safetyThresholds.MAX_DCM_P_GAIN);
    c_.dcmIntegralGain = clamp(i, 0., c_.safetyThresholds.MAX_DCM_I_GAIN);
    c_.dcmDerivGain = clamp(d, 0., c_.safetyThresholds.MAX_DCM_D_GAIN);
  }

  inline void dcmIntegratorTimeConstant(double dcmIntegratorTimeConstant) noexcept
  {
    c_.dcmIntegratorTimeConstant = dcmIntegratorTimeConstant;
    dcmIntegrator_.timeConstant(dcmIntegratorTimeConstant);
  }

  inline void dcmDerivatorTimeConstant(double dcmDerivatorTimeConstant) noexcept
  {
    c_.dcmDerivatorTimeConstant = dcmDerivatorTimeConstant;
    dcmDerivator_.timeConstant(dcmDerivatorTimeConstant);
  }

  inline void extWrenchSumLowPassCutoffPeriod(double cutoffPeriod) noexcept
  {
    c_.extWrench.extWrenchSumLowPassCutoffPeriod = cutoffPeriod;
    extWrenchSumLowPass_.cutoffPeriod(cutoffPeriod);
  }

  inline void comOffsetLowPassCutoffPeriod(double cutoffPeriod) noexcept
  {
    c_.extWrench.comOffsetLowPassCutoffPeriod = cutoffPeriod;
    comOffsetLowPass_.cutoffPeriod(cutoffPeriod);
  }

  inline void comOffsetLowPassCoMCutoffPeriod(double cutoffPeriod) noexcept
  {
    c_.extWrench.comOffsetLowPassCoMCutoffPeriod = cutoffPeriod;
    comOffsetLowPassCoM_.cutoffPeriod(cutoffPeriod);
  }

  inline void comOffsetDerivatorTimeConstant(double timeConstant) noexcept
  {
    c_.extWrench.comOffsetDerivatorTimeConstant = timeConstant;
    comOffsetDerivator_.timeConstant(timeConstant);
  }

  inline void comWeight(double weight) noexcept
  {
    c_.comWeight = weight;
    comTask->weight(weight);
  }

  inline void comStiffness(const Eigen::Vector3d & stiffness) noexcept
  {
    c_.comStiffness = stiffness;
    comTask->stiffness(stiffness);
  }

  inline void contactWeight(double weight) noexcept
  {
    c_.contactWeight = weight;
    for(auto footT : contactTasks)
    {
      footT->weight(c_.contactWeight);
    }
  }

  inline void contactStiffness(const sva::MotionVecd & stiffness) noexcept
  {
    c_.contactStiffness = stiffness;
    for(auto contactT : contactTasks)
    {
      contactT->stiffness(stiffness);
    }
  }

  inline void contactDamping(const sva::MotionVecd & damping) noexcept
  {
    c_.contactDamping = damping;
    for(auto contactT : contactTasks)
    {
      contactT->damping(damping);
    }
  }

  inline void copAdmittance(const Eigen::Vector2d & copAdmittance) noexcept
  {
    c_.copAdmittance = clamp(copAdmittance, 0., c_.safetyThresholds.MAX_COP_ADMITTANCE);
    for(auto contactT : contactTasks)
    {
      contactT->admittance(contactAdmittance());
    }
  }

  inline void copMaxVel(const sva::MotionVecd & copMaxVel) noexcept
  {
    c_.copMaxVel = copMaxVel;
    for(const auto & footTask : footTasks)
    {
      footTask.second->maxLinearVel(copMaxVel.linear());
      footTask.second->maxAngularVel(copMaxVel.angular());
    }
  }

  /* Set the gain of the low-pass velocity filter of the cop tasks */
  inline void copVelFilterGain(double gain) noexcept
  {
    c_.copVelFilterGain = mc_filter::utils::clamp(gain, 0, 1);
    for(auto & ft : footTasks)
    {
      ft.second->velFilterGain(gain);
    }
  }

  /* Get the gain of the low-pass velocity filter of the cop tasks */
  inline double copVelFilterGain() const noexcept
  {
    return c_.copVelFilterGain;
  }

  inline void vdcFrequency(double freq) noexcept
  {
    c_.vdcFrequency = clamp(freq, 0., 10.);
  }

  inline void vdcStiffness(double stiffness) noexcept
  {
    c_.vdcStiffness = clamp(stiffness, 0., 1e4);
  }

  inline void dfzAdmittance(double dfzAdmittance) noexcept
  {
    c_.dfzAdmittance = clamp(dfzAdmittance, 0., c_.safetyThresholds.MAX_DFZ_ADMITTANCE);
  }

  inline void dfzDamping(double dfzDamping) noexcept
  {
    c_.dfzDamping = clamp(dfzDamping, 0., c_.safetyThresholds.MAX_DFZ_DAMPING);
  }

  inline void fdqpWeights(const FDQPWeights & fdqp) noexcept
  {
    c_.fdqpWeights = fdqp;
  }

  /**
   * @brief Changes the safety thresholds
   *
   * This ensures that all the parameters depending on these safety parameters
   * are within the new thresholds. If they are out of bounds, they will be
   * clamped back to the new range, and a warning message will be displayed.
   *
   * @param thresholds New safety thresholds
   */
  inline void safetyThresholds(const SafetyThresholds & thresholds) noexcept
  {
    c_.safetyThresholds = thresholds;
    c_.clampGains();
    // only requried because we want to apply the new gains immediately
    copAdmittance(c_.copAdmittance);
  }

  /**
   * @brief Changes the parameters of the DCM bias estimator.
   *
   * @param biasConfig Configuration parameters for the bias estimation
   */
  inline void dcmBiasEstimatorConfiguration(const DCMBiasEstimatorConfiguration & biasConfig) noexcept
  {
    auto & bc = c_.dcmBias;
    bc = biasConfig;
    dcmEstimator_.setBiasDriftPerSecond(bc.biasDriftPerSecondStd);
    dcmEstimator_.setDcmMeasureErrorStd(bc.dcmMeasureErrorStd);
    dcmEstimator_.setZmpMeasureErrorStd(bc.zmpMeasureErrorStd);
    dcmEstimator_.setBiasLimit(bc.biasLimit);
  }

  /** @brief Get parameters of the DCM bias estimator. */
  inline const DCMBiasEstimatorConfiguration & dcmBiasEstimatorConfiguration() const noexcept
  {
    return c_.dcmBias;
  }

  /**
   * @brief Changes the parameters for the external wrenches.
   *
   * @param extWrenchConfig Configuration parameters for the external wrenches
   */
  inline void externalWrenchConfiguration(const ExternalWrenchConfiguration & extWrenchConfig) noexcept
  {
    c_.extWrench = extWrenchConfig;
    extWrenchSumLowPass_.cutoffPeriod(c_.extWrench.extWrenchSumLowPassCutoffPeriod);
    comOffsetLowPass_.cutoffPeriod(c_.extWrench.comOffsetLowPassCutoffPeriod);
    comOffsetLowPassCoM_.cutoffPeriod(c_.extWrench.comOffsetLowPassCoMCutoffPeriod);
    comOffsetDerivator_.timeConstant(c_.extWrench.comOffsetDerivatorTimeConstant);
  }

  /** @brief Get the parameters for the external wrenches. */
  inline const ExternalWrenchConfiguration & externalWrenchConfiguration() const noexcept
  {
    return c_.extWrench;
  }

private:
  void dimWeight(const Eigen::VectorXd & dimW) override;
  Eigen::VectorXd dimWeight() const override;

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & activeJointsName,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs = {}) override;

  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & unactiveJointsName,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs = {}) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  /**
   * @brief Add contact to the task (set tasks target, update support area).
   * This function does not immediately add contact tasks to the solver, this
   * will be done by update() when the task is added to the solver.
   */
  void addContact(ContactState contactState, const internal::Contact & contact);

  /** Check whether the robot is in the air. */
  void checkInTheAir();

  /** Computes the ratio of force distribution between the feet based on
   * the reference ZMP and contact ankle positions.
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

  /** Compute desired wrench based on DCM error. */
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
   */
  void updateFootForceDifferenceControl();

  /** Update ZMP frame from contact state. */
  void updateZMPFrame();

  /** Get 6D contact admittance vector from 2D CoP admittance. */
  inline sva::ForceVecd contactAdmittance() const noexcept
  {
    return {{c_.copAdmittance.y(), c_.copAdmittance.x(), 0.}, {0., 0., 0.}};
  }

  inline void zmpcc(const ZMPCCConfiguration & zmpccConfig) noexcept
  {
    c_.zmpcc = zmpccConfig;
    zmpcc_.configure(zmpccConfig);
  }

  /** @brief External wrench. */
  struct ExternalWrench
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /// Target (expected) external wrench
    sva::ForceVecd target;
    /// Measured external wrench
    sva::ForceVecd measured;
    /// Gain of measured external wrench
    sva::MotionVecd gain;
    /// Name of the surface to which the external wrench is applied
    std::string surfaceName;
  };

  /** @brief Compute the CoM offset and the sum wrench from the external wrenches.
   *
   *  @tparam TargetOrMeasured Change depending on the used wrenches
   *  @param robot Robot used to transform surface wrenches (control robot or real robot)
   */
  template<sva::ForceVecd ExternalWrench::*TargetOrMeasured>
  Eigen::Vector3d computeCoMOffset(const mc_rbdyn::Robot & robot) const;

  /** @brief Compute the sum of external wrenches.
   *
   *  @tparam TargetOrMeasured Change depending on the used wrenches
   *  @param robot Robot used to transform surface wrenches (control robot or real robot)
   *  @param com Robot CoM
   */
  template<sva::ForceVecd ExternalWrench::*TargetOrMeasured>
  sva::ForceVecd computeExternalWrenchSum(const mc_rbdyn::Robot & robot, const Eigen::Vector3d & com) const;

  /** @brief Compute the position, force, and moment of the external contacts in the world frame.
   *
   *  @param [in] robot Robot (control robot or real robot)
   *  @param [in] surfaceName Surface name
   *  @param [in] surfaceWrench Surface wrench
   *  @param [out] pos Position of the external contact in the world frame
   *  @param [out] force Force of the external contact in the world frame
   *  @param [out] moment Moment of the external contact in the world frame
   */
  void computeExternalContact(const mc_rbdyn::Robot & robot,
                              const std::string & surfaceName,
                              const sva::ForceVecd & surfaceWrench,
                              Eigen::Vector3d & pos,
                              Eigen::Vector3d & force,
                              Eigen::Vector3d & moment) const;

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

  /**
   * @brief Actual configuration of the stabilizer.
   * Called when reconfigure_ is true
   *
   * @param solver Solver to which this task has been added
   */
  void configure_(mc_solver::QPSolver & solver);

  /** Ensures that the configuration is valid */
  void checkConfiguration(const StabilizerConfiguration & config);

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
  std::unordered_map<ContactState,
                     internal::Contact,
                     EnumClassHash,
                     std::equal_to<ContactState>,
                     Eigen::aligned_allocator<std::pair<const ContactState, internal::Contact>>>
      contacts_;
  std::vector<ContactState> addContacts_; /**< Contacts to add to the QPSolver when the task is inserted */
  std::unordered_map<ContactState, std::shared_ptr<mc_tasks::force::CoPTask>, EnumClassHash> footTasks;
  std::vector<std::shared_ptr<mc_tasks::force::CoPTask>> contactTasks; /** Foot tasks for the established contacts */
  std::vector<std::string> contactSensors; /** Force sensors corresponding to established contacts */

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
  Eigen::Vector3d comTargetRaw_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comdTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d comddTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d zmpdTarget_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmTarget_ = Eigen::Vector3d::Zero();
  double omega_;

  double t_ = 0.; /**< Time elapsed since the task is running */

protected:
  /**< Default (user-provided) configuration for the stabilizer. This configuration is superseeded by the parameters set
   * in the GUI */
  StabilizerConfiguration defaultConfig_;
  /**< Last valid stabilizer configuration. */
  StabilizerConfiguration lastConfig_;
  /**< Stabilizer configuration to disable. */
  StabilizerConfiguration disableConfig_;
  /**< Online stabilizer configuration, can be set from the GUI. Defaults to defaultConfig_ */
  StabilizerConfiguration c_;
  /**< Whether the stabilizer needs to be reconfigured at the next
   * update(solver) call */
  bool reconfigure_ = true;
  bool enabled_ = true; /** Whether the stabilizer is enabled */

  Eigen::QuadProgDense qpSolver_; /**< Least-squares solver for wrench distribution */
  Eigen::Vector3d dcmAverageError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dcmVelError_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoM_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredCoMd_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredZMP_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d measuredDCM_ = Eigen::Vector3d::Zero(); /// Measured DCM (only used for logging)
  Eigen::Vector3d measuredDCMUnbiased_ = Eigen::Vector3d::Zero(); /// DCM unbiased (only used for logging)
  Eigen::Vector3d measuredCoMUnbiased_ = Eigen::Vector3d::Zero(); /// CoM unbiased (only used for logging)
  sva::ForceVecd measuredNetWrench_ = sva::ForceVecd::Zero();

  bool zmpccOnlyDS_ = true; /**< Only apply ZMPCC in double support */
  ZMPCC zmpcc_; /**< Compute CoM offset due to ZMPCC compensation */

  /**
   * Filtering of the divergent component of motion (DCM)
   * and estimation of a bias betweeen the DCM and the corresponding zero moment point for a linearized inverted
   * pendulum model.
   */
  stateObservation::LipmDcmEstimator dcmEstimator_;
  /**< Whether the estimator needs to be reset (robot in the air, initialization) */
  bool dcmEstimatorNeedsReset_ = true;

  /** @name Members related to stabilization in the presence of external wrenches
   *
   *  Adding an offset to the CoM for the predictable / measurable external wrenches on the robot surface.
   *  @{
   */
  std::vector<ExternalWrench> extWrenches_;
  sva::ForceVecd extWrenchSumTarget_ = sva::ForceVecd::Zero(); /**< Sum of target (expected) external wrenches */
  sva::ForceVecd extWrenchSumMeasured_ = sva::ForceVecd::Zero(); /**< Sum of measured external wrenches */
  Eigen::Vector3d comOffsetTarget_ = Eigen::Vector3d::Zero(); /**< Target (expected) CoM offset */
  Eigen::Vector3d comOffsetMeasured_ = Eigen::Vector3d::Zero(); /**< Measured CoM offset */
  Eigen::Vector3d comOffsetErr_ = Eigen::Vector3d::Zero(); /**< CoM offset error */
  Eigen::Vector3d comOffsetErrCoM_ = Eigen::Vector3d::Zero(); /**< CoM offset error handled by CoM modification */
  Eigen::Vector3d comOffsetErrZMP_ = Eigen::Vector3d::Zero(); /**< CoM offset error handled by ZMP modification */
  mc_filter::LowPass<sva::ForceVecd>
      extWrenchSumLowPass_; /**< Low-pass filter of the sum of the measured external wrenches */
  mc_filter::LowPass<Eigen::Vector3d> comOffsetLowPass_; /**< Low-pass filter of CoM offset */
  mc_filter::LowPass<Eigen::Vector3d>
      comOffsetLowPassCoM_; /**< Low-pass filter of CoM offset to extract CoM modification */
  mc_filter::StationaryOffset<Eigen::Vector3d> comOffsetDerivator_; /**< Derivator of CoM offset */
  /** @} */

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
  sva::ForceVecd desiredWrench_ = sva::ForceVecd::Zero(); /**< Result of the DCM feedback */
  sva::ForceVecd distribWrench_ = sva::ForceVecd::Zero(); /**< Result of the force distribution QP */
  Eigen::Vector3d distribZMP_ =
      Eigen::Vector3d::Zero(); /**< ZMP corresponding to force distribution result (desired ZMP) */
  sva::PTransformd zmpFrame_ = sva::PTransformd::Identity(); /**< Frame in which the ZMP is computed */
};

extern template Eigen::Vector3d StabilizerTask::computeCoMOffset<&StabilizerTask::ExternalWrench::target>(
    const mc_rbdyn::Robot &) const;
extern template Eigen::Vector3d StabilizerTask::computeCoMOffset<&StabilizerTask::ExternalWrench::measured>(
    const mc_rbdyn::Robot &) const;

extern template sva::ForceVecd StabilizerTask::computeExternalWrenchSum<&StabilizerTask::ExternalWrench::target>(
    const mc_rbdyn::Robot &,
    const Eigen::Vector3d &) const;
extern template sva::ForceVecd StabilizerTask::computeExternalWrenchSum<&StabilizerTask::ExternalWrench::measured>(
    const mc_rbdyn::Robot &,
    const Eigen::Vector3d &) const;

} // namespace lipm_stabilizer
} // namespace mc_tasks
