#pragma once

#include <mc_rbdyn/CompoundJointConstraintDescription.h>
#include <mc_rbdyn/Robots.h>
#include <mc_solver/ConstraintSet.h>

#include <Tasks/QPSolver.h>

namespace mc_solver
{

using CompoundJointConstraintDescription = mc_rbdyn::CompoundJointConstraintDescription;

namespace details
{

/** Implement compound joint constraints
 *
 * This constraint is given a set of CompoundJointConstraint for a given robot
 * and will enforce them
 */
struct CompoundJointConstraint : public tasks::qp::ConstraintFunction<tasks::qp::Inequality>
{
  MC_SOLVER_DLLAPI CompoundJointConstraint(const mc_rbdyn::Robots & robots,
                                           unsigned int rIndex,
                                           double dt,
                                           const std::vector<CompoundJointConstraintDescription> & desc = {});

  MC_SOLVER_DLLAPI ~CompoundJointConstraint() override;

  MC_SOLVER_DLLAPI void addConstraint(const mc_rbdyn::Robots & robots,
                                      unsigned int rIndex,
                                      const CompoundJointConstraintDescription & desc);

  MC_SOLVER_DLLAPI void updateNrVars(const std::vector<rbd::MultiBody> & mbs,
                                     const tasks::qp::SolverData & data) override;

  MC_SOLVER_DLLAPI void update(const std::vector<rbd::MultiBody> & mbs,
                               const std::vector<rbd::MultiBodyConfig> & mbcs,
                               const tasks::qp::SolverData & data) override;

  inline const Eigen::MatrixXd & AInEq() const override
  {
    return A_;
  }

  inline int maxInEq() const override
  {
    return static_cast<int>(descs_.size());
  }

  inline std::string nameInEq() const override
  {
    return name_;
  }

  inline const Eigen::VectorXd & bInEq() const override
  {
    return b_;
  }

  std::string descInEq(const std::vector<rbd::MultiBody> & mbs, int i) override;

private:
  size_t rIndex_;
  std::string name_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  // Constant part of b_
  Eigen::VectorXd b_cst_;
  // Timestep
  double dt_;
  // Simplified form of the description
  struct Desc
  {
    // Index of j1 in mbc
    size_t q1Idx;
    // Index of j1 in matrix
    int q1MatIdx;
    // Index of j2 in mbc
    size_t q2Idx;
    // Index of j2 in matrix
    int q2MatIdx;
    // x index of p1
    double p1_x;
    // y index of p1
    double p1_y;
    // P_x = p2_x - p1_x
    double P_x;
    // P_y = p2_y - p1_y
    double P_y;
  };
  std::vector<Desc> descs_;
};

} // namespace details

/** Wrapper for mc_solver::details::CompundJointConstraint */
struct MC_SOLVER_DLLAPI CompoundJointConstraint : public ConstraintSet
{
  CompoundJointConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double dt);

  void addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) override;

  void removeFromSolver(tasks::qp::QPSolver & solver) override;

private:
  details::CompoundJointConstraint constr_;
};

} // namespace mc_solver
