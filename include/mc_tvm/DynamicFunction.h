/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/LinearFunction.h>

#include <RBDyn/Jacobian.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_tvm
{

/** Implement the equation of motion for a given robot.
 *
 * It can be given contacts that will be integrated into the equation of
 * motion (\see DynamicFunction::addContact).
 *
 * It manages the force variables related to these contacts.
 *
 * Notably, it does not take care of enforcing Newton 3rd law of motion when
 * two actuated robots are in contact.
 *
 */
struct MC_TVM_DLLAPI DynamicFunction : public tvm::function::abstract::LinearFunction
{
public:
  using Output = tvm::function::abstract::LinearFunction::Output;
  DISABLE_OUTPUTS(Output::JDot)
  SET_UPDATES(DynamicFunction, Jacobian, B)

  /** Construct the equation of motion for a given robot */
  DynamicFunction(const mc_rbdyn::Robot & robot);

  /** Add a 3d contact to the function
   *
   * This adds forces variables for every contact point belonging to the
   * robot of this dynamic function.
   *
   * \param frame Contact frame
   *
   * \param points Contact points in the frame's parent body's frame
   *
   * \param dir Contact direction
   *
   * Returns the force variables that were created by this contact
   */
  const tvm::VariableVector & addContact3d(const mc_rbdyn::RobotFrame & frame,
                                           std::vector<sva::PTransformd> points,
                                           double dir);

  /** Add a surface contact to the function
   *
   * This adds a 6d wrench variable for the surface contact.
   *
   * \param frame Contact frame
   *
   * \param dir Contact direction
   *
   * Returns the wrench variable that was created by this contact
   */
  const tvm::VariablePtr & addContact6d(const mc_rbdyn::RobotFrame & frame, double dir);

  /** Removes the contact associated to the given frame
   *
   * \param frame Contact frame
   */
  void removeContact(const mc_rbdyn::RobotFrame & frame);

  /** Returns the contact force at the given contact frame
   *
   * \param f Contact frame
   *
   * \throws If no contact has been added with that frame
   */
  sva::ForceVecd contactForce(const mc_rbdyn::RobotFrame & f) const;

protected:
  void updateb();

  const mc_rbdyn::Robot & robot_;

  /** Holds data for the force part of the motion equation */
  struct ForceContact
  {
    /** Constructor */
    ForceContact(const mc_rbdyn::RobotFrame & frame, std::vector<sva::PTransformd> points, double dir);

    /** Update jacobians */
    void updateJacobians(DynamicFunction & parent);

    /** Compute the contact force */
    sva::ForceVecd force() const;

    /** Associated frame */
    mc_rbdyn::ConstRobotFramePtr frame_;

    /** Force associated to a contact */
    tvm::VariableVector forces_;

    /** Contact points */
    std::vector<sva::PTransformd> points_;

    /** Contact direction */
    double dir_;

    /** RBDyn jacobian */
    rbd::Jacobian jac_;
    /** RBDyn jacobian blocks */
    rbd::Blocks blocks_;

    /** Used for intermediate Jacobian computation */
    Eigen::MatrixXd force_jac_;
    Eigen::MatrixXd full_jac_;
  };

  /** Holds data for the contact wrenches part of the motion equation */
  struct WrenchContact
  {
    /** Constructor for 6D wrench */
    WrenchContact(const mc_rbdyn::RobotFrame & frame, double dir);

    /** Update jacobian */
    void updateWrenchJacobian(DynamicFunction & parent);

    /** Return the contact wrench */
    sva::ForceVecd wrench() const;

    /** Associated frame */
    mc_rbdyn::ConstRobotFramePtr frame_;

    /** 6D wrench var associated to a contact */
    tvm::VariablePtr wrench_;

    /** Contact direction */
    double dir_;

    /** RBDyn jacobian */
    rbd::Jacobian jac_;
    /** RBDyn jacobian blocks */
    rbd::Blocks blocks_;

    /** Used for intermediate Jacobian computation */
    Eigen::MatrixXd full_jac_;
  };

  std::vector<ForceContact> contactForces_;

  std::vector<ForceContact>::const_iterator findContactForce(const mc_rbdyn::RobotFrame & frame) const;

  std::vector<WrenchContact> contactWrenches_;

  std::vector<WrenchContact>::const_iterator findContactWrench(const mc_rbdyn::RobotFrame & frame) const;

  void updateJacobian();
};

using DynamicFunctionPtr = std::shared_ptr<DynamicFunction>;

} // namespace mc_tvm
