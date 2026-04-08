#pragma once

#include <mc_rtc/Configuration.h>
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

using CompoundJointConstraintDescriptionVector =
    std::vector<CompoundJointConstraintDescription, Eigen::aligned_allocator<CompoundJointConstraintDescription>>;

} // namespace mc_rbdyn

namespace mc_rtc
{
template<>
struct ConfigurationLoader<mc_rbdyn::CompoundJointConstraintDescription>
{
  static mc_rbdyn::CompoundJointConstraintDescription load(const mc_rtc::Configuration & config)
  {
    return mc_rbdyn::CompoundJointConstraintDescription{config("j1"), config("j2"), config("p1"), config("p2")};
  }
  static mc_rtc::Configuration save(const mc_rbdyn::CompoundJointConstraintDescription & desc)
  {
    mc_rtc::Configuration config;
    config.add("j1", desc.j1);
    config.add("j2", desc.j1);
    config.add("p1", desc.p1);
    config.add("p2", desc.p2);
    return config;
  }
};
} // namespace mc_rtc
