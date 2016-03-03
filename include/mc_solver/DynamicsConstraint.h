#pragma once

#include <mc_solver/KinematicsConstraint.h>

#include <Tasks/QPMotionConstr.h>

namespace mc_solver
{

/** \class DynamicsConstraint
 * Holds dynamics constraints for a robot
 */

struct MC_SOLVER_DLLAPI DynamicsConstraint : public KinematicsConstraint
{
public:
  /** Constructor
   * Builds a regular joint limits constraint and a motion constr depending on
   * the nature of the robot
   * See tasks::qp::MotionConstr for details on the latter one
   * \param robots The robots including the robot affected by this constraint
   * \param robotIndex The index of the robot affected by this constraint
   * \param timeStep Solver timestep
   * \param infTorque If true, ignore the torque limits set in the robot model
   */
  DynamicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool infTorque = false);

  /** Constructor
   * Builds a damped joint limits constraint and a motion constr depending on
   * the nature of the robot
   * See tasks::qp::MotionConstr for details on the latter one
   * \param robots The robots including the robot affected by this constraint
   * \param robotIndex The index of the robot affected by this constraint
   * \param timeStep Solver timestep
   * \param damper Value of the damper {interaction distance, safety distance,
   * offset}
   * \param velocityPercent Maximum joint velocity percentage, 0.5 is advised
   * \param infTorque If true, ignore the torque limits set in the robot model
   */
  DynamicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, const std::array<double, 3> & damper, double velocityPercent = 1.0, bool infTorque = false);

  /** Implementation of mc_solver::ConstraintSet::addToSolver */
  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const override;

  /** Implementation of mc_solver::ConstraintSet::removeFromSolver */
  virtual void removeFromSolver(tasks::qp::QPSolver & solver) const override;
public:
  /** Classic motion constraint, may be null **/
  std::shared_ptr<tasks::qp::MotionConstr> motionConstr;
  /** Motion constraint used for robots with flexibilities in their model, may
   * be null
   */
  std::shared_ptr<tasks::qp::MotionSpringConstr> motionSpringConstr;
public:
  /** \deprecated{Default constructor, not made for general usage}
   */
  DynamicsConstraint() {}
private:
  /** Private function to build the proper MotionConstr */
  void build_constr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, bool infTorque);
};

} // namespace mc_solver
