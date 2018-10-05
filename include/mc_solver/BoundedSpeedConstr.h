#pragma once

#include <mc_solver/QPSolver.h>
#include <mc_solver/api.h>

#include <Tasks/QPConstr.h>

namespace mc_solver
{
/** \class BoundedSpeedConstr
 * \brief Wrapper around tasks::qp::BoundedSpeedConstr
 */

struct MC_SOLVER_DLLAPI BoundedSpeedConstr : public ConstraintSet
{
public:
  BoundedSpeedConstr(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt);

  virtual void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) override;

  virtual void removeFromSolver(tasks::qp::QPSolver & solver) override;

  void addBoundedSpeed(QPSolver & solver,
                       const std::string & bodyName,
                       const Eigen::Vector3d & bodyPoint,
                       const Eigen::MatrixXd & dof,
                       const Eigen::VectorXd & speed);

  void addBoundedSpeed(QPSolver & solver,
                       const std::string & bodyName,
                       const Eigen::Vector3d & bodyPoint,
                       const Eigen::MatrixXd & dof,
                       const Eigen::VectorXd & lowerSpeed,
                       const Eigen::VectorXd & upperSpeed);

  bool removeBoundedSpeed(QPSolver & solver, const std::string & bodyName);

  size_t nrBoundedSpeeds() const;

  void reset(QPSolver & solver);

private:
  std::shared_ptr<tasks::qp::BoundedSpeedConstr> constr;
  unsigned int robotIndex;
};

} // namespace mc_solver
