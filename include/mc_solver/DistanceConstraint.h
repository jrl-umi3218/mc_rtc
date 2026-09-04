/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/ConstraintSet.h>

#include <mc_rbdyn/DistanceLimit.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/void_ptr.h>

namespace mc_solver
{

struct QPSolver;

/** \class DistanceConstraint
 *
 * Creates a distance constraint manager between two robots.
 *
 * If the two robots are the same, this effectively creates a self-distance constraint (e.g. self-collision).
 *
 * Each distance limit is assigned an internal identity based on its two convex names and
 * its distance constraint type. The identity is suffixed with:
 * - "_min" when iDist > sDist, this is minimum distance constraint mostly used for collision avoidance
 * - "_max" otherwise, this is maximum distance constraint when two links should not move too far apart
 *
 * This allows minimum-distance and maximum-distance constraints between the same
 * pair of convexes to coexist as distinct distance constraints.
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

  /** Remove a distance limit between two convexes
   *
   * The distance limit identity is determined from the two convex names and the
   * distance constraint type (see __keyByNames).
   *
   * \param solver The solver into which this constraint was added
   * \param dl The distance limit to remove
   * \return True if the distance limit was found and removed, false otherwise
   */
  bool removeDistanceLimit(QPSolver & solver, const mc_rbdyn::DistanceLimit & dl);

  /** Remove a set of distance limits
   *
   * \param solver The solver into which this constraint was added
   * \param dls List of distance limits to remove
   */
  void removeDistanceLimits(QPSolver & solver, const std::vector<mc_rbdyn::DistanceLimit> & dls);

  /** Remove all distance limits between two bodies
   *
   * This removes every distance limit between the specified pair of bodies,
   * including both minimum-distance and maximum-distance constraints.
   *
   * \param solver The solver into which this constraint was added
   * \param b1Name Name of the first body
   * \param b2Name Name of the second body
   * \return True if at least one distance limit was removed, false otherwise
   */
  bool removeDistanceLimitByBody(QPSolver & solver, const std::string & b1Name, const std::string & b2Name);

  /** Add a distance limit represented by mc_rbdyn::DistanceLimit
   *
   * The distance limit object is allowed to specify wildcard names to add multiple
   * distance limits at once, if body1 is named bodyA* and body2 is named bodyB*
   * then distance constraints will be added for all convex objects in robot1
   * (resp. robot2) that start with bodyA (resp. bodyB)
   *
   * The distance limit identity is based on the two convex names and the distance
   * constraint type:
   * - "_min" is appended when iDist > sDist
   * - "_max" is appended otherwise
   *
   * Therefore, two distance limits involving the same pair of convexes can coexist
   * when one is a minimum-distance constraint and the other is a
   * maximum-distance constraint.
   *
   * \param solver The solver into which this constraint was added
   * \param dl The distance limit that should be added
   */
  void addDistanceLimit(QPSolver & solver, const mc_rbdyn::DistanceLimit & dl);

  /** Add a set of distance limits
   *
   * \see addDistanceLimit for details on wildcard distance limit specification and
   * distance limit identity.
   *
   * \param solver The solver into which this constraint was added
   * \param dls The set of distance limits that should be added
   */
  void addDistanceLimits(QPSolver & solver, const std::vector<mc_rbdyn::DistanceLimit> & dls);

  /** Returns true if a distance limit between the given pair of convexes is in this constraint.
   *
   * This checks the convex names only and does not distinguish between
   * "_min" and "_max" distance limit identities.
   */
  bool hasDistanceLimit(const std::string & c1, const std::string & c2) const noexcept;

  /** Remove all distance limits from the constraint */
  void reset();

  /** Get the automated monitoring setting */
  inline bool automaticMonitor() const noexcept { return autoMonitor_; }

  /** Set the automated monitoring setting
   *
   * If true, distance limit monitors are automatically added/removed depending on the distance limit activation.
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
   * - tasks::qp::DistanceConstr
   *
   * In TVM backend:
   * - details::TVMDistanceConstraint
   */
  mc_rtc::void_ptr constraint_;

  /** Index of the first robot affected by the constraint */
  unsigned int r1Index;

  /** Index of the second robot affected by the constraint */
  unsigned int r2Index;

  /** Current set of distance limits */
  std::vector<mc_rbdyn::DistanceLimit> dls;

private:
  /** Internal state used to manage distance limit identities */
  int dlId;

  /** Maps a distance limit identity to its internal ID and distance limit data */
  std::map<std::string, std::pair<int, mc_rbdyn::DistanceLimit>> dlIdDict;

  /** Build the unique identity key for a distance limit.
   *
   * The key is composed of the two convex names followed by:
   * - "_min" when iDist > sDist
   * - "_max" otherwise
   */
  std::string __keyByNames(const mc_rbdyn::DistanceLimit & dl);

  /** Create an internal ID for a distance limit.
   *
   * Returns -1 if a distance limit with the same identity already exists.
   */
  int __createDistanceLimitId(const mc_rbdyn::DistanceLimit & dl);

  /** Remove and return the internal ID and distance limit data associated with a distance limit.
   *
   * The distance limit identity is computed using __keyByNames.
   */
  std::pair<int, mc_rbdyn::DistanceLimit> __popDistanceLimitId(const mc_rbdyn::DistanceLimit & dl);

  /** Actually adds the distance limit to the constraint, handles ID creation and wildcard support */
  void __addDistanceLimit(mc_solver::QPSolver & solver, const mc_rbdyn::DistanceLimit & dl);

  /* Internal management for distance limit display */
  bool autoMonitor_ = true;
  std::unordered_set<int> monitored_;
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_;
  std::vector<std::string> category_;

  void addMonitorButton(int dlId, const mc_rbdyn::DistanceLimit & dl);
  void toggleDistanceLimitMonitor(int dlId, const mc_rbdyn::DistanceLimit * dl = nullptr);
};

} // namespace mc_solver
