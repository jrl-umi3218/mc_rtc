/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <Eigen/Core>

namespace mc_rbdyn
{

inline Eigen::Matrix3d hat(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d ret;
  // clang-format off
  ret << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
  // clang-format on
  return ret;
}

} // namespace mc_rbdyn
