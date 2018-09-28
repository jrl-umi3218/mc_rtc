#pragma once

#include <mc_solver/ConstraintSet.h>

#include <Tasks/QPContactConstr.h>
#include <Tasks/QPMotionConstr.h>

namespace mc_solver
{

/** \class ContactConstraint
 * \brief Wraps multiple constraints related to Contact
 */

struct MC_SOLVER_DLLAPI ContactConstraint : public ConstraintSet
{
public:
  /** This enum list the possible types of ContactConstr, see Tasks
   * documentation for mathematical details on each of them
   */
  enum ContactType
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
   * \param timeStep Ignored if contactType is Acceleration
   * \param contactType Type of contact corresponding, this value dictates the
   * type of tasks::qp::ContactConstr that will be generated
   * \param dynamics If true, the constraint will include a
   * tasks::qp::PositiveLambda constraint in addition to the
   * tasks::qp::ContactConstr
   */
  ContactConstraint(double timeStep, ContactType contactType = Velocity, bool dynamics = true);

  /** Implementation of mc_solver::ConstraintSet::addToSolver */
  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) override;

  /** Implementation of mc_solver::ConstraintSet::removeFromSolver */
  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;

public:
  /** Holds the proper type of ContactConstr based on constructor input, always
   * holds a valid pointer */
  std::shared_ptr<tasks::qp::ContactConstr> contactConstr;
  /** May be a null-ptr */
  std::shared_ptr<tasks::qp::PositiveLambda> posLambdaConstr;

public:
  /** \deprecated{Default constructor, not made for general usage}
   */
  ContactConstraint() {}
};

} // namespace mc_solver
