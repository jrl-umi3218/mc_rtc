/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/ConstraintSet.h>

#include <mc_rtc/void_ptr.h>

#include <Tasks/QPContactConstr.h>

namespace mc_solver
{

/** \class ContactConstraint
 *
 * Handle geometric constraints on contacts
 */
struct MC_SOLVER_DLLAPI ContactConstraint : public ConstraintSet
{
public:
  /** This enum list the possible types of ContactConstr, see Tasks
   * documentation for mathematical details on each of them
   */
  enum class ContactType
  {
    /** Acceleration constraint */
    Acceleration = 0,
    /** Velocity constraint */
    Velocity = 1,
    /** Position constraint */
    Position = 2
  };

public:
  /** Constructor
   *
   * \param timeStep Solver timestep
   *
   * \param contactType Enable different geometric constraint
   *
   */
  ContactConstraint(double timeStep, ContactType contactType = ContactType::Velocity);

  /** Implementation of mc_solver::ConstraintSet::addToSolver */
  void addToSolverImpl(QPSolver & solver) override;

  /** Implementation of mc_solver::ConstraintSet::removeFromSolver */
  void removeFromSolverImpl(QPSolver & solver) override;

  /** Returns the underlying constraint in the Tasks backend
   *
   * \throws If the backend is not Tasks backend
   */
  tasks::qp::ContactConstr * contactConstr();

  /** Returns the contact type for this constraint */
  ContactType contactType() const noexcept { return contactType_; }

private:
  /** Holds the contact constraint implementation
   *
   * In Tasks backend:
   * - tasks::qp::ContactConstr
   *
   * No-op in TVM backend
   */
  mc_rtc::void_ptr constraint_;
  ContactType contactType_;
};

} // namespace mc_solver
