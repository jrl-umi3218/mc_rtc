/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_solver/ConstraintSet.h>

#include <Tasks/QPConstr.h>

#include <set>

namespace mc_solver
{

struct QPSolver;

/** \class CollisionsConstraint
 * \brief Create a collision constraint between two robots. If the two robots
 * are the same, this effectivly creates a self-collision constraint
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

  /** Implementation of mc_solver::ConstraintSet::addToSolver */
  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) override;

  /** Implementation of mc_solver::ConstraintSet::removeFromSolver */
  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;

public:
  /** The actual collision constraint object */
  std::shared_ptr<tasks::qp::CollisionConstr> collConstr;
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
  void __addCollision(const mc_solver::QPSolver & solver, const mc_rbdyn::Collision & col);

  /* Internal management for collision display */
  std::unordered_set<int> monitored_;
  std::shared_ptr<mc_rtc::gui::StateBuilder> gui_;
  std::vector<std::string> category_;
  void addMonitorButton(int collId, const mc_rbdyn::Collision & col);
  void toggleCollisionMonitor(int collId);

  bool inSolver_ = false;

public:
  /** \deprecated{Default constructor, not made for general usage} */
  CollisionsConstraint() {}
};

/** \class RobotEnvCollisionsConstraint
 *
 * Utility class for the very common-case of (robot,env) scenarios. This class
 * manages both (robot,robot) and (robot,env) collisions and provides utility
 * functions to add/remove collisions to both
 */

struct MC_SOLVER_DLLAPI RobotEnvCollisionsConstraint : public ConstraintSet
{
public:
  /* Constructor
   *
   * Assumes that you wish to create the constraint with robots.robot() and
   * robots.env()
   * \param robots The robots that will be affected by this constraint
   * \param timeStep Timestep of the solver
   */
  RobotEnvCollisionsConstraint(const mc_rbdyn::Robots & robots, double timeStep);

  /** Remove a collision between the robot and the environment
   * \param solver The solver into which this constraint was added
   * \param rBodyName Robot's convex name
   * \param eBodyName Env's convex name
   * \return True if a collision was removed, false otherwise
   */
  bool removeEnvCollision(QPSolver & solver, const std::string & rBodyName, const std::string & eBodyName);

  /** Remove a collision between the robot and the environment based on body
   * names
   * \param solver The solver into which this constraint was added
   * \param rBodyName Robot's body name
   * \param eBodyName Env's body name
   * \return True if at least one collision was removed, false otherwise
   */
  bool removeEnvCollisionByBody(QPSolver & solver, const std::string & rBodyName, const std::string & eBodyName);

  /** Remove a collision between the robot and the robot
   * \param solver The solver into which this constraint was added
   * \param body1Name Robot's convex name
   * \param body2Name Robot's convex name
   * \return True if a collision was removed, false otherwise
   */
  bool removeSelfCollision(QPSolver & solver, const std::string & body1Name, const std::string & body2Name);

  /** Add a (robot, env) collision represented by mc_rbdyn::Collision
   * \param solver The solver into which this constraint was added
   * \param col The collision that should be added
   */
  void addEnvCollision(QPSolver & solver, const mc_rbdyn::Collision & col);

  /** Add a (robot, robot) collision represented by mc_rbdyn::Collision
   * \param solver The solver into which this constraint was added
   * \param col The collision that should be added
   */
  void addSelfCollision(QPSolver & solver, const mc_rbdyn::Collision & col);

  /** Based on the existing set of (robot, env) collisions, a set of contact
   * concerning the robot and a set of collisions:
   *   - remove Collision that are not in the new set
   *   - remove Collision if they contradict the Contact set
   *   - keep or add Collision in the new set
   * \param solver The solver into which this constraint was added
   * \param contacts Set of contacts affecting the robot
   * \param cols Set of collision to add
   */
  void setEnvCollisions(QPSolver & solver,
                        const std::vector<mc_rbdyn::Contact> & contacts,
                        const std::vector<mc_rbdyn::Collision> & cols);

  /** Based on the existing set of (robot, robot) collisions, a set of contact
   * concerning the robot and a set of collisions:
   *   - remove Collision that are not in the new set
   *   - remove Collision if they contradict the Contact set
   *   - keep or add Collision in the new set
   * \param solver The solver into which this constraint was added
   * \param contacts Set of contacts affecting the robot
   * \param cols Set of collision to add
   */
  void setSelfCollisions(QPSolver & solver,
                         const std::vector<mc_rbdyn::Contact> & contacts,
                         const std::vector<mc_rbdyn::Collision> & cols);

  /** Implementation of mc_solver::ConstraintSet::addToSolver */
  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) override;

  /** Implementation of mc_solver::ConstraintSet::removeFromSolver */
  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;

public:
  /** (robot, robot) collision constraint */
  CollisionsConstraint selfCollConstrMng;
  /** (robot, env) collision constraint */
  CollisionsConstraint envCollConstrMng;

private:
  std::set<std::string> __bodiesFromContacts(const mc_rbdyn::Robot & robot,
                                             const std::vector<mc_rbdyn::Contact> & contacts);
};

} // namespace mc_solver
