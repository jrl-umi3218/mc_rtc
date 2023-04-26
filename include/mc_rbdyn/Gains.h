/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <Eigen/Core>

namespace mc_rbdyn
{

/** Wrapper around fixed-sized Eigen vectors that are used for gains in the framework
 *
 * Their purpose is to allow code such as:
 * ```cpp
 * my_task.gain = 3.0;
 * ```
 * Where `my_task.gain` is a fixed vector of size 2, to mean the same as:
 * ```cpp
 * my_task.gain = Eigen::Vector2d::Constant(3.0);
 * ```
 */
template<int N>
struct Gains : public Eigen::Matrix<double, N, 1>
{
  static_assert(N > 0, "This is only usable for fixed-size gains");

  using Eigen::Matrix<double, N, 1>::Matrix;

  Gains(double value) { this->setConstant(value); }
};

using Gains2d = Gains<2>;
using Gains3d = Gains<3>;
using Gains6d = Gains<6>;

} // namespace mc_rbdyn
