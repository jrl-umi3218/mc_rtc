/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

/** This test does not run anything, it merely check that new constraints can be derived the Constraint utility class */

#include <mc_solver/EqualityConstraint.h>
#include <mc_solver/GenInequalityConstraint.h>
#include <mc_solver/InequalityConstraint.h>

#define COMMON_OVERRIDE                      \
  const Eigen::MatrixXd & A() const override \
  {                                          \
    return A_;                               \
  }                                          \
  void compute() override {}                 \
  Eigen::MatrixXd A_;

#define EQ_OVERRIDE(NAME)                      \
  int maxEq() const override                   \
  {                                            \
    return 1;                                  \
  }                                            \
  std::string nameEq() const override          \
  {                                            \
    return #NAME;                              \
  }                                            \
  const Eigen::VectorXd & bEq() const override \
  {                                            \
    return b_;                                 \
  }                                            \
  Eigen::VectorXd b_;

#define INEQ_OVERRIDE(NAME)                      \
  int maxInEq() const override                   \
  {                                              \
    return 1;                                    \
  }                                              \
  std::string nameInEq() const override          \
  {                                              \
    return #NAME;                                \
  }                                              \
  const Eigen::VectorXd & bInEq() const override \
  {                                              \
    return b_;                                   \
  }                                              \
  Eigen::VectorXd b_;

#define GENINEQ_OVERRIDE(NAME)                          \
  int maxGenInEq() const override                       \
  {                                                     \
    return 1;                                           \
  }                                                     \
  std::string nameGenInEq() const override              \
  {                                                     \
    return #NAME;                                       \
  }                                                     \
  const Eigen::VectorXd & LowerGenInEq() const override \
  {                                                     \
    return L_;                                          \
  }                                                     \
  const Eigen::VectorXd & UpperGenInEq() const override \
  {                                                     \
    return U_;                                          \
  }                                                     \
  Eigen::VectorXd L_;                                   \
  Eigen::VectorXd U_;

#define CONSTRAINT_OVERRIDE_COMMON(NAME, BASE) \
  struct NAME : public BASE                    \
  {

#define CONSTRAINT_Robot_CONSTRUCTOR(NAME, BASE) \
  NAME(unsigned int rIndex) : BASE(rIndex) {}

#define CONSTRAINT_Lambda_CONSTRUCTOR(NAME, BASE) \
  NAME(const tasks::qp::ContactId & cid) : BASE(cid) {}

#define CONSTRAINT_Force_CONSTRUCTOR(NAME, BASE) \
  NAME(const mc_solver::QPSolver & solver, const tasks::qp::ContactId & cid) : BASE(solver, cid) {}

#define CONSTRAINT_END \
  }                    \
  ;

#define WRITE_CONSTRAINT(NAME, BASE, CTOR_MACRO, OVERRIDE_MACRO) \
  CONSTRAINT_OVERRIDE_COMMON(NAME, BASE)                         \
  CTOR_MACRO(NAME, BASE)                                         \
  COMMON_OVERRIDE                                                \
  OVERRIDE_MACRO(NAME)                                           \
  CONSTRAINT_END

#define WRITE_ALL_CONSTRAINTS_TYPE(BASE, OVERRIDE_MACRO)                                                       \
  WRITE_CONSTRAINT(Test##BASE##Robot, mc_solver::BASE##Robot, CONSTRAINT_Robot_CONSTRUCTOR, OVERRIDE_MACRO)    \
  WRITE_CONSTRAINT(Test##BASE##Lambda, mc_solver::BASE##Lambda, CONSTRAINT_Lambda_CONSTRUCTOR, OVERRIDE_MACRO) \
  WRITE_CONSTRAINT(Test##BASE##Force, mc_solver::BASE##Force, CONSTRAINT_Force_CONSTRUCTOR, OVERRIDE_MACRO)

#define WRITE_ALL_CONSTRAINTS                                     \
  WRITE_ALL_CONSTRAINTS_TYPE(EqualityConstraint, EQ_OVERRIDE)     \
  WRITE_ALL_CONSTRAINTS_TYPE(InequalityConstraint, INEQ_OVERRIDE) \
  WRITE_ALL_CONSTRAINTS_TYPE(GenInequalityConstraint, GENINEQ_OVERRIDE)

WRITE_ALL_CONSTRAINTS

int main()
{
  unsigned int rIndex = 0;
  tasks::qp::ContactId cid{};
  mc_solver::QPSolver solver;
  {
    TestEqualityConstraintRobot c(rIndex);
  }
  {
    TestInequalityConstraintRobot c(rIndex);
  }
  {
    TestGenInequalityConstraintRobot c(rIndex);
  }
  {
    TestEqualityConstraintLambda c(cid);
  }
  {
    TestInequalityConstraintLambda c(cid);
  }
  {
    TestGenInequalityConstraintLambda c(cid);
  }
  {
    TestEqualityConstraintForce c(solver, cid);
  }
  {
    TestInequalityConstraintForce c(solver, cid);
  }
  {
    TestGenInequalityConstraintForce c(solver, cid);
  }
  return 0;
}
