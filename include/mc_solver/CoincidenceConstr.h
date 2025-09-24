#pragma once

#include <mc_rtc/void_ptr.h>
#include <mc_solver/ConstraintSet.h>
#include <Tasks/QPCoincidenceConstr.h>

namespace mc_rbdyn
{
struct Robots;
}

namespace mc_solver
{

struct QPSolver;

/** \class CoincidenceConstraint
 *
 * Constraint to maintain coincidence between two points on a robot
 */
struct MC_SOLVER_DLLAPI CoincidenceConstraint : public ConstraintSet
{
public:
  /** Constructor
   *
   * \param robots The robots for which the constraint will apply
   * \param name1 Name of the first link
   * \param name1 Name of the second link
   * \param type Type of the joint
   * \param dt Time step of the control
   */
  CoincidenceConstraint(const mc_rbdyn::Robots & robots,
                        std::string name1,
                        std::string name2,
                        std::string type,
                        const Eigen::VectorXd & joints,
                        double dt);

  void addToSolverImpl(QPSolver & solver) override;

  void removeFromSolverImpl(QPSolver & solver) override;

public:
  /** Holds the constraint implementation */
  std::string name1_;
  std::string name2_;
  std::string type_;
  Eigen::VectorXd joints_;
  double dt_;

  tasks::qp::CoincidenceConstr * coincidenceConstr();

private:
  mc_rtc::void_ptr constraint_;
};

} // namespace mc_solver
