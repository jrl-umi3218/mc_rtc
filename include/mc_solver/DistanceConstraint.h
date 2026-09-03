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

/** \class DistanceConstraint
 *
 * Creates a collision constraint manager between two robots.
 *
 * If the two robots are the same, this effectively creates a self-collision constraint
 *
 * Each collision is assigned an internal identity based on its two convex names and
 * its distance constraint type. The identity is suffixed with:
 * - "_min" when iDist > sDist
 * - "_max" otherwise
 *
 * This allows minimum-distance and maximum-distance constraints between the same
 * pair of convexes to coexist as distinct collisions.
 */
struct MC_SOLVER_DLLAPI DistanceConstraint : public ConstraintSet
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
   * \param timeStep Time step of the control
   */
  DistanceConstraint(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index, double timeStep);

  /** Remove a collision between two convexes
   *
   * The collision identity is determined from the two convex names and the
   * distance constraint type (see __keyByNames).
   *
   * \param solver The solver into which this constraint was added
   * \param col The collision to remove
   * \return True if the collision was found and removed, false otherwise
   */
  bool removeCollision(QPSolver & solver, const mc_rbdyn::Collision & col);

  /** Remove a set of collisions
   *
   * \param solver The solver into which this constraint was added
   * \param cols List of collisions to remove
   */
  void removeCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols);

  /** Remove all collisions between two bodies
   *
   * This removes every collision between the specified pair of bodies,
   * including both minimum-distance and maximum-distance collision constraints.
   *
   * \param solver The solver into which this constraint was added
   * \param b1Name Name of the first body
   * \param b2Name Name of the second body
   * \return True if at least one collision was removed, false otherwise
   */
  bool removeCollisionByBody(QPSolver & solver, const std::string & b1Name, const std::string & b2Name);

  /** Add a collision represented by mc_rbdyn::Collision
   *
   * The collision object is allowed to specify wildcard names to add multiple
   * collisions at once, if body1 is named bodyA* and body2 is named bodyB*
   * then collision constraints will be added for all convex objects in robot1
   * (resp. robot2) that start with bodyA (resp. bodyB)
   *
   * The collision identity is based on the two convex names and the distance
   * constraint type:
   * - "_min" is appended when iDist > sDist
   * - "_max" is appended otherwise
   *
   * Therefore, two collisions involving the same pair of convexes can coexist
   * when one is a minimum-distance constraint and the other is a
   * maximum-distance constraint.
   *
   * \param solver The solver into which this constraint was added
   * \param col The collision that should be added
   */
  void addCollision(QPSolver & solver, const mc_rbdyn::Collision & col);

  /** Add a set of collisions
   *
   * \see addCollision for details on wildcard collision specification and
   * collision identity.
   *
   * \param solver The solver into which this constraint was added
   * \param cols The set of collisions that should be added
   */
  void addCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols);

  /** Returns true if a collision between the given pair of convexes is in this constraint.
   *
   * This checks the convex names only and does not distinguish between
   * "_min" and "_max" collision identities.
   */
  bool hasCollision(const std::string & c1, const std::string & c2) const noexcept;

  /** Remove all collisions from the constraint */
  void reset();

  /** Get the automated monitoring setting */
  inline bool automaticMonitor() const noexcept { return autoMonitor_; }

  /** Set the automated monitoring setting
   *
   * If true, collision monitors are automatically added/removed depending on the collision activation.
   *
   * If false, monitors are managed by the user
   */
  inline void automaticMonitor(bool a) noexcept { autoMonitor_ = a; }

  void addToSolverImpl(QPSolver & solver) override;

  void update(QPSolver & solver) override;

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

  /** Current set of collisions */
  std::vector<mc_rbdyn::Collision> cols;

private:
  /** Internal state used to manage collision identities */
  int collId;

  /** Maps a collision identity to its internal ID and collision data */
  std::map<std::string, std::pair<int, mc_rbdyn::Collision>> collIdDict;

  /** Build the unique identity key for a collision.
   *
   * The key is composed of the two convex names followed by:
   * - "_min" when iDist > sDist
   * - "_max" otherwise
   */
  std::string __keyByNames(const mc_rbdyn::Collision & col);

  /** Create an internal ID for a collision.
   *
   * Returns -1 if a collision with the same identity already exists.
   */
  int __createCollId(const mc_rbdyn::Collision & col);

  /** Remove and return the internal ID and collision data associated with a collision.
   *
   * The collision identity is computed using __keyByNames.
   */
  std::pair<int, mc_rbdyn::Collision> __popCollId(const mc_rbdyn::Collision & col);

  /** Actually adds the collision to the constraint, handles ID creation and wildcard support */
  void __addCollision(mc_solver::QPSolver & solver, const mc_rbdyn::Collision & col);

  /* Internal management for collision display */
  bool autoMonitor_ = true;
  std::unordered_set<int> monitored_;
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_;
  std::vector<std::string> category_;

  void addMonitorButton(int collId, const mc_rbdyn::Collision & col);
  void toggleCollisionMonitor(int collId, const mc_rbdyn::Collision * col = nullptr);
};

} // namespace mc_solver
