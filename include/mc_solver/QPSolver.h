#ifndef _H_MCCONTROLQPSOLVER_H_
#define _H_MCCONTROLQPSOLVER_H_

#include <mc_solver/api.h>
#include <mc_solver/ConstraintSet.h>

#include <mc_control/msg/QPResult.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/robot.h>

#include <Tasks/QPSolver.h>

#include <memory>

namespace mc_solver
{

/** \class QPSolver
 *
 * Wraps a tasks::qp::QPSolver instance
 *
 * Always ensure that the solver is up-to-date
 */

struct MC_SOLVER_DLLAPI QPSolver
{
public:
  /** Constructor
   * \param robot Set of robots managed by this solver
   * \param timeStep Timestep of the solver
   */
  QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep);

  /** Add a constraint set
   * \param cs Constraint set added to the solver
   */
  void addConstraintSet(const ConstraintSet & cs);

  /** Remove a constraint set
   * \param cs Constrain set removed from the solver
   */
  void removeConstraintSet(const ConstraintSet & cs);

  /** Add a task to the solver
   * \param task Pointer to the added task, QPSolver does not take ownership of this pointer and the caller should make sure the object remains valid until it is removed from the solver
   */
  void addTask(tasks::qp::Task * task);

  /** Remove a task from the solver
   * \param task Pointer to the removed task. The task is not deleted after being removed
   */
  void removeTask(tasks::qp::Task * task);

  /** Add a constraint function from the solver
   * \param constraint Pointer to the ConstraintFunction. QPSolver does not take ownserhip of this pointer and the caller should make sure the object remains valid until it is removed from the solver
   */
  template<typename ... Fun>
  void addConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->addToSolver(robots().mbs(), solver);
  }

  /** Remove a constraint function from the solver
   * \param constraint Pointer to the constraint that will be removed. It is not destroyed afterwards
   */
  template<typename ... Fun>
  void removeConstraint(tasks::qp::ConstraintFunction<Fun...> * constraint)
  {
    constraint->removeFromSolver(solver);
  }

  /** Gives access to the tasks::qp::BilateralContact entity in the solver from a contact id
   * \param id The contact id of the contact
   * \return The tasks:qp::BilateralContact entity from the solver if id is valid, otherwise, the first element of the pair is -1 and the reference is invalid
   */
  std::pair<int, const tasks::qp::BilateralContact&> contactById(const tasks::qp::ContactId & id);

  /** Reset all contacts in the solver and use the new set of contacts provided
   * \item contact Set of mc_rbdyn::Contact
   */
  void setContacts(const std::vector<mc_rbdyn::Contact> & contacts);

  /** Run one iteration of the QP.
   *
   * If succesful, will update the robots' configurations
   * \return True if successful, false otherwise.
   */
  bool run();

  /** Provides the result of run() for robots.robot()
   * \param curTime Unused
   */
  const mc_control::QPResultMsg & send(double curTime = 0);

  /** Gives access to the main robot in the solver */
  const mc_rbdyn::Robot & robot() const;
  /** Gives access to the main robot in the solver */
  mc_rbdyn::Robot & robot();

  /** Gives access to the environment robot in the solver (see mc_rbdyn::Robots) */
  const mc_rbdyn::Robot & env() const;
  /** Gives access to the environment robot in the solver (see mc_rbdyn::Robots) */
  mc_rbdyn::Robot & env();

  /** Gives access to the robots controlled by this solver */
  const mc_rbdyn::Robots & robots() const;
  /** Gives access to the robots controlled by this solver */
  mc_rbdyn::Robots & robots();

  /** Update constraints matrix sizes
   *
   * \note This is mainly provided to allow safe usage of raw constraint from
   * Tasks rather than those wrapped in this library, you probably do not need
   * to call this
   */
  void updateConstrSize(); private: std::shared_ptr<mc_rbdyn::Robots> robots_p;
  double timeStep;

  /** Holds unilateral contacts in the solver */
  std::vector<tasks::qp::UnilateralContact> uniContacts;
  /** Holds bilateral contacts in the solver */
  std::vector<tasks::qp::BilateralContact> biContacts;

private:
  /** The actual solver instance */
  tasks::qp::QPSolver solver;
  /** Latest result */
  mc_control::QPResultMsg qpRes;

  /** Update qpRes from the latest run() */
  void __fillResult();
public:
  /** \deprecated{Default constructor, not made for general usage} */
  QPSolver() {}
};

}

#endif
