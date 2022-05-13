/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/ConstraintSet.h>

#include <mc_rbdyn/Collision.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/void_ptr.h>

namespace mc_solver
{

struct QPSolver;

/** \class CollisionsConstraint
 *
 * Creates a collision constraint manager between two robots.
 *
 * If the two robots are the same, this effectively creates a self-collision constraint
 */
struct MC_SOLVER_DLLAPI CollisionsConstraint : public ConstraintSet
{
public:
  /** Default value of damping offset */
  constexpr static double defaultDampingOffset = 0.1;

public:
  /** Constructor
   *
   * \param robots The robots for which the constraint will apply
   * \param r1Index Index of the first robot affected by the constraint
   * \param r2Index Index of the second robot affected by the constraint
   * \param Timestep timeStep of the control
   */
  CollisionsConstraint(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index, double timeStep);

  /** Remove a collision between two convexes
   * \param solver The solver into which this constraint was added
   * \param b1Name Name of the first convex
   * \param b2Name Name of the second convex
   * \return True if the collision was found and removed, false otherwise
   */
  bool removeCollision(QPSolver & solver, const std::string & b1Name, const std::string & b2Name);

  /** Remove a set of collisions
   *
   * \param solver The solver into which this constraint was added
   *
   * \param cols List of collisions to remove
   */
  void removeCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols);

  /** Remove all collisions between two bodies
   * \param solver The solver into which this constraint was added
   * \param b1Name Name of the first body
   * \param b2Name Name of the second body
   * \return True if at least one collision was removed, false otherwise
   */
  bool removeCollisionByBody(QPSolver & solver, const std::string & byName, const std::string & b2Name);

  /** Add a collision represented by mc_rbdyn::Collision
   *
   * The collision object is allowed to specify wildcard names to add multiple
   * collisions at once, if body1 is named bodyA* and body2 is named bodyB*
   * then collision constraints will be added for all convex objects in robot1
   * (resp. robot2) that start with bodyA (resp. bodyB)
   *
   * \param solver The solver into which this constraint was added \param col
   * The collision that should be added
   */
  void addCollision(QPSolver & solver, const mc_rbdyn::Collision & col);

  /** Add a set of collisions
   *
   * \see addCollision for details on wildcard collision specification
   *
   * \param solver The solver into which this constraint was added
   * \param cols The set of collisions that should be added
   */
  void addCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols);

  /** Remove all collisions from the constraint */
  void reset();

  void addToSolverImpl(QPSolver & solver) override;

  void removeFromSolverImpl(QPSolver & solver) override;

public:
  /** Holds the constraint implementation
   *
   * In Tasks backend:
   * - tasks::qp::CollisionConstr
   *
   * In TVM backend:
   * - details::TVMCollisionConstraint
   */
  mc_rtc::void_ptr constraint_;
  /** Index of the first robot affected by the constraint */
  unsigned int r1Index;
  /** Index of the second robot affected by the constraint */
  unsigned int r2Index;
  /** Curent set of collisions */
  std::vector<mc_rbdyn::Collision> cols;

private:
  /* Internal sauce to manage collisions */
  int collId;
  std::map<std::string, std::pair<int, mc_rbdyn::Collision>> collIdDict;
  std::string __keyByNames(const std::string & name1, const std::string & name2);
  int __createCollId(const mc_rbdyn::Collision & col);
  std::pair<int, mc_rbdyn::Collision> __popCollId(const std::string & name1, const std::string & name2);
  /** Actually adds the collision to the constraint, handles id creation and wildcard support */
  void __addCollision(mc_solver::QPSolver & solver, const mc_rbdyn::Collision & col);

  /* Internal management for collision display */
  std::unordered_set<int> monitored_;
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_;
  std::vector<std::string> category_;
  void addMonitorButton(int collId, const mc_rbdyn::Collision & col);
  void toggleCollisionMonitor(int collId);
};

} // namespace mc_solver
