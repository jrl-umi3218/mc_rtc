#pragma once

#include <Eigen/Core>
#include <string>

namespace mc_rbdyn
{

/** Describe a compound joint constraint
 *
 * This defines:
 * 1. a pair of joint names (q1, q2)
 * 2. a pair of 2D points (p1, p2)
 *
 * The constraint is that:
 * \f[
 *  (q - p1) \times (p2 - p1) \leqslant 0
 * \f]
 *
 * Where \f$ q = \begin{bmatrix} q1 q2 \end{bmatrix} \f$
 */
struct CompoundJointConstraintDescription
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::string j1;
  std::string j2;
  Eigen::Vector2d p1;
  Eigen::Vector2d p2;
};

} // namespace mc_rbdyn
