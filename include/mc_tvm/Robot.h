/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/CoM.h>
#include <mc_tvm/Limits.h>
#include <mc_tvm/Momentum.h>

#include <mc_rbdyn/Robot.h>

#include <tvm/Variable.h>
#include <tvm/VariableVector.h>
#include <tvm/graph/abstract/Node.h>

#include <RBDyn/FD.h>

#include <mc_rbdyn/ExternalTorqueSensor.h>

namespace mc_tvm
{

/** Represent a robot managed by the optimization problem
 *
 * It is created through an \ref mc_rbdyn::Robot
 *
 * It provides signals that are relevant for computing quantities related to the robot.
 *
 * Variables:
 * - q (split between free-flyer and joints)
 * - tau (see Outputs)
 *
 * Individual outputs:
 *
 * - FK: forward kinematics (computed by RBDyn::FK)
 * - FV: forward velocity (computed by RBDyn::FV), depends on FK
 * - FA: forward acceleration (computed by RBDyn::FA), depends on FV
 * - NormalAcceleration: update bodies' normal acceleration, depends on FV
 * - tau: generalized torque vector
 * - H: inertia matrix signal, depends on FV
 * - C: non-linear effect vector signal (Coriolis, gravity, external forces), depends on FV
 *
 * Meta outputs:
 *   These outputs are provided for convenience sake
 * - Geometry: depends on FK
 * - Dynamics: depends on FA + normalAcceleration (i.e. everything)
 *
 */
struct MC_TVM_DLLAPI Robot : public tvm::graph::abstract::Node<Robot>
{
  SET_OUTPUTS(Robot, FK, FV, FA, NormalAcceleration, tau, H, C, ExternalForces)
  SET_UPDATES(Robot, FK, FV, FA, NormalAcceleration, H, C, ExternalForces)

  friend struct mc_rbdyn::Robot;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using mimic_variables_t = std::pair<tvm::VariableVector, Eigen::VectorXd>;

protected:
  struct NewRobotToken
  {
  };

public:
  Robot(NewRobotToken, const mc_rbdyn::Robot & robot);

  Robot(Robot &&) = delete;
  Robot & operator=(Robot &&) = delete;

  Robot(const Robot &) = delete;
  Robot & operator=(const Robot &) = delete;

  /** Retrieve the associated robot (const)
   *
   * \throws If the original robot has been destroyed
   */
  inline const mc_rbdyn::Robot & robot() const { return robot_; }

  /** Retrieve the joint limits (const) */
  inline const Limits & limits() const noexcept { return limits_; }

  /** Retrieve the joint limits */
  inline Limits & limits() noexcept { return limits_; }

  /** Access q variable (const) */
  inline const tvm::VariablePtr & q() const noexcept { return q_; }
  /** Access q variable */
  inline tvm::VariablePtr & q() noexcept { return q_; }

  /** Access q first derivative (joint velocity) (const) */
  inline const tvm::VariablePtr & alpha() const noexcept { return dq_; }
  /** Access q first derivative (joint velocity) */
  inline tvm::VariablePtr & alpha() noexcept { return dq_; }

  /** Access q second derivative (joint acceleration) (const) */
  inline const tvm::VariablePtr & alphaD() const noexcept { return ddq_; }
  /** Access q second derivative (joint acceleration) */
  inline tvm::VariablePtr & alphaD() noexcept { return ddq_; }
  /** Access joint acceleration from external forces (const) */
  inline const Eigen::VectorXd & alphaDExternal() const noexcept { return ddq_ext_; }
  /** Access joint acceleration from external forces */
  inline Eigen::VectorXd & alphaDExternal() noexcept { return ddq_ext_; }

  /** Access floating-base variable (const) */
  inline const tvm::VariablePtr & qFloatingBase() const noexcept { return q_fb_; }
  /** Access free-flyer variable */
  inline tvm::VariablePtr & qFloatingBase() noexcept { return q_fb_; }

  /** Access joints variable (const) */
  inline const tvm::VariablePtr & qJoints() const noexcept { return q_joints_; }
  /** Access joints variable */
  inline tvm::VariablePtr & qJoints() noexcept { return q_joints_; }

  /** Given a joint index creates a TVM variable corresponding to this variable
   *
   * \warning This returns a different object (but the same variable from TVM pov) each time this function is called
   *
   * \param jIdx Joint index
   *
   * \throws If the joint index is out of bounds or the joint is not actuated
   */
  tvm::VariablePtr qJoint(size_t jIdx);

  /** Given a joint name, returns the TVM variable it belongs to and the index of the joint in this variable
   *
   * \param jName Joint name
   *
   * \throws If the robot does not have such a joint or the joint is not actuated
   */
  inline tvm::VariablePtr qJoint(const std::string & jName) { return qJoint(robot().jointIndexByName(jName)); }

  /** Access mimics' variable map (const) */
  inline const std::map<tvm::VariablePtr, mimic_variables_t> & mimics() const noexcept { return mimics_; }
  /** Access mimics' variable map */
  inline std::map<tvm::VariablePtr, mimic_variables_t> & mimics() noexcept { return mimics_; }

  /** Access tau variable (const) */
  inline const tvm::VariablePtr & tau() const noexcept { return tau_; }
  /** Access tau variable */
  inline tvm::VariablePtr & tau() { return tau_; }
  /** Access tau external variable (const) */
  inline const Eigen::VectorXd & tauExternal() const noexcept { return tau_ext_; }
  /** Access tau external variable */
  inline Eigen::VectorXd & tauExternal() { return tau_ext_; }

  /** Returns the CoM algorithm associated to this robot (const) */
  inline const CoM & comAlgo() const noexcept { return *com_; }

  /** Returns the CoM algorithm associated to this robot */
  inline CoM & comAlgo() noexcept { return *com_; }

  /** Returns the momentum algorithm associated with this robot (const) */
  inline const Momentum & momentumAlgo() const noexcept { return *momentum_; }

  /** Returns the momentum algorithm associated with this robot (const) */
  inline Momentum & momentumAlgo() noexcept { return *momentum_; }

  /** Returns the mass matrix */
  inline const Eigen::MatrixXd & H() const noexcept { return fd_.H(); }

  /** Returns the non-linear dynamics component */
  inline const Eigen::VectorXd & C() const noexcept { return fd_.C(); }

  /** Vector of normal acceleration in body coordinates */
  inline const std::vector<sva::MotionVecd> & normalAccB() const noexcept { return normalAccB_; }

  /** Given a joint index in the reference joint order, returns the corresponding joint index in the q variable
   *
   * @note Joint indices can be -1 for joints present in refJointOrder but not
   * in the robot's q (such as filtered joints in some robot modules)
   *
   * @param jointIndex Joint index in refJointOrder
   *
   * @returns joint index in q
   *
   */
  inline Eigen::DenseIndex refJointIndexToQIndex(size_t jointIndex) const
  {
    return refJointIndexToQIndex_.at(jointIndex);
  }

  /** Given a joint index in the reference joint order, returns the corresponding joint index in the q derivatives
   * variable
   *
   * @note Joint indices can be -1 for joints present in refJointOrder but not
   * in the robot's q (such as filtered joints in some robot modules)
   *
   * @param jointIndex Joint index in refJointOrder
   *
   * @returns joint index in q
   *
   */
  inline Eigen::DenseIndex refJointIndexToQDotIndex(size_t jointIndex) const
  {
    return refJointIndexToQDotIndex_.at(jointIndex);
  }

private:
  /** Parent instance */
  const mc_rbdyn::Robot & robot_;
  /** Joint limits */
  Limits limits_;
  /** Floating-base variable */
  tvm::VariablePtr q_fb_;
  /** Joints variable */
  tvm::VariablePtr q_joints_;
  /** Generalized configuration variable */
  tvm::VariablePtr q_;
  /** Map mimic leader joint to their followers */
  std::map<tvm::VariablePtr, mimic_variables_t> mimics_;
  /** Derivative of q */
  tvm::VariablePtr dq_;
  /** Double derivative of q */
  tvm::VariablePtr ddq_;
  /** Joint acceleration from external forces */
  Eigen::VectorXd ddq_ext_;
  /** Tau variable */
  tvm::VariablePtr tau_;
  /** Tau external variable */
  Eigen::VectorXd tau_ext_;
  /** Normal accelerations of the bodies */
  std::vector<sva::MotionVecd> normalAccB_;
  /** Forward dynamics algorithm associated to this robot */
  rbd::ForwardDynamics fd_;
  /** CoM algorithm of this robot */
  CoMPtr com_;
  /** Momentum algorithm of this robot */
  MomentumPtr momentum_;
  /** Correspondance between refJointOrder index and q index. **/
  std::vector<Eigen::DenseIndex> refJointIndexToQIndex_;
  /** Correspondance between refJointOrder index and q dot index. **/
  std::vector<Eigen::DenseIndex> refJointIndexToQDotIndex_;

  /* Update functions */
  void updateFK();
  void updateFV();
  void updateFA();
  void updateNormalAcceleration();
  void updateH();
  void updateC();
  void updateEF();
};

} // namespace mc_tvm
