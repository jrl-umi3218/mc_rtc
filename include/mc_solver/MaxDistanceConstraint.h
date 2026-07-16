/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/ConstraintSet.h>

#include <mc_rbdyn/MaxDist.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/void_ptr.h>

namespace mc_solver
{

struct QPSolver;

/** \class MaxDistanceConstraint
 *
 * Creates a max distance constraint manager between two robots.
 *
 * If the two robots are the same, this effectively creates a self-max-distance constraint
 */
struct MC_SOLVER_DLLAPI MaxDistanceConstraint : public ConstraintSet
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
  MaxDistanceConstraint(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index, double timeStep);

  /** Remove a max distance between two convexes
   * \param solver The solver into which this constraint was added
   * \param b1Name Name of the first convex
   * \param b2Name Name of the second convex
   * \return True if the max distance was found and removed, false otherwise
   */
  bool removeMaxDist(QPSolver & solver, const std::string & b1Name, const std::string & b2Name);

  /** Remove a set of max distances
   *
   * \param solver The solver into which this constraint was added
   *
   * \param maxDists List of max distances to remove
   */
  void removeMaxDists(QPSolver & solver, const std::vector<mc_rbdyn::MaxDist> & maxDists);

  /** Remove all max distances between two bodies
   * \param solver The solver into which this constraint was added
   * \param b1Name Name of the first body
   * \param b2Name Name of the second body
   * \return True if at least one max distance was removed, false otherwise
   */
  bool removeMaxDistByBody(QPSolver & solver, const std::string & byName, const std::string & b2Name);

  /** Add a max distance represented by mc_rbdyn::MaxDist
   *
   * The max distance object is allowed to specify wildcard names to add multiple
   * max distances at once, if body1 is named bodyA* and body2 is named bodyB*
   * then max distance constraints will be added for all convex objects in robot1
   * (resp. robot2) that start with bodyA (resp. bodyB)
   *
   * \param solver The solver into which this constraint was added \param maxD
   * The max distance that should be added
   */
  void addMaxDist(QPSolver & solver, const mc_rbdyn::MaxDist & maxD);

  /** Add a set of max distances
   *
   * \see addMaxDist for details on wildcard max distance specification
   *
   * \param solver The solver into which this constraint was added
   * \param maxDists The set of max distances that should be added
   */
  void addMaxDists(QPSolver & solver, const std::vector<mc_rbdyn::MaxDist> & maxD);

  /** Returns true if the given max distance is in this constraint */
  bool hasMaxDist(const std::string & c1, const std::string & c2) const noexcept;

  /** Remove all max distances from the constraint */
  void reset();

  /** Get the automated monitoring setting */
  inline bool automaticMonitor() const noexcept { return autoMonitor_; }

  /** Set the automated monitoring setting
   *
   * If true, monitors are automatically added/removed depending on the max distance activation
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
   * - https://github.com/anastasiabolotnikova/robogami_controller
   *
   * In TVM backend:
   * - details::TVMMaxDistConstraint
   */
  mc_rtc::void_ptr constraint_;
  /** Index of the first robot affected by the constraint */
  unsigned int r1Index;
  /** Index of the second robot affected by the constraint */
  unsigned int r2Index;
  /** Curent set of max distanes */
  std::vector<mc_rbdyn::MaxDist> maxDists;

private:
  /* Internal sauce to manage max-distances */
  int maxDistId;
  std::map<std::string, std::pair<int, mc_rbdyn::MaxDist>> maxDistIdDict;
  std::string __keyByNames(const std::string & name1, const std::string & name2);
  int __createMaxDId(const mc_rbdyn::MaxDist & maxD);
  std::pair<int, mc_rbdyn::MaxDist> __popMaxDistId(const std::string & name1, const std::string & name2);
  /** Actually adds the max distance to the constraint, handles id creation and wildcard support */
  void __addMaxDist(mc_solver::QPSolver & solver, const mc_rbdyn::MaxDist & maxD);

  /* Internal management for max-distances display */
  bool autoMonitor_ = true;
  std::unordered_set<int> monitored_;
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_;
  std::vector<std::string> category_;
  void addMonitorButton(int maxDId, const mc_rbdyn::MaxDist & maxD);
  void toggleMaxDistMonitor(int maxDId, const mc_rbdyn::MaxDist * maxD = nullptr);
};

} // namespace mc_solver
