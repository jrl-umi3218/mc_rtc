/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#include <mc_rtc/logging.h>
#include <mc_solver/utils/ContactWrenchMatrixToLambdaMatrix.h>

#if not EIGEN_VERSION_AT_LEAST(3, 2, 90)
namespace Eigen
{
using Index = Eigen::DenseIndex;
}
#endif

namespace mc_solver
{

namespace utils
{

ContactWrenchMatrixToLambdaMatrix::ContactWrenchMatrixToLambdaMatrix(const mc_solver::QPSolver & solver,
                                                                     const tasks::qp::ContactId & id)
{
  auto qp_c = solver.contactById(id);
  if(qp_c.first == -1)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[ContactWrenchMatrixToLambdaMatrix] Provided contact has not been added to the solver yet");
  }
  const auto & contact = qp_c.second;
  const auto & cones = contact.r1Cones;
  const auto & points = contact.r1Points;
  size_t nLambda = 0;
  Eigen::MatrixXd Gi = Eigen::MatrixXd::Zero(6, 0);
  for(const auto & c : cones)
  {
    nLambda += c.generators.size();
    Gi.resize(3, std::max<Eigen::Index>(Gi.cols(), static_cast<Eigen::Index>(c.generators.size())));
  }
  transform_.resize(6, static_cast<int>(nLambda));
  Eigen::MatrixXd X_cf_pi_T = Eigen::MatrixXd::Zero(6, 3);
  int col = 0;
  for(size_t i = 0; i < points.size(); ++i)
  {
    sva::PTransformd X_b_pi{contact.r1Points[i]};
    sva::PTransformd X_cf_pi = X_b_pi * contact.X_b1_cf.inv();
    X_cf_pi_T.block(0, 0, 3, 3) = sva::vector3ToCrossMatrix(X_cf_pi.translation()) * X_cf_pi.rotation().transpose();
    X_cf_pi_T.block(3, 0, 3, 3) = X_cf_pi.rotation().transpose();
    for(size_t j = 0; j < cones[i].generators.size(); ++j)
    {
      Gi.col(static_cast<int>(j)) = cones[i].generators[j];
    }
    transform_.block(0, col, 6, static_cast<int>(cones[i].generators.size())) =
        X_cf_pi_T * Gi.block(0, 0, 3, static_cast<int>(cones[i].generators.size()));
    col += static_cast<int>(cones[i].generators.size());
  }
}

} // namespace utils

} // namespace mc_solver
