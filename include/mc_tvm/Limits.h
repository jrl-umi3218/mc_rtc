/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <Eigen/Core>

namespace mc_tvm
{

/** Simple joint limits description extracted from a RobotModule */
struct MC_TVM_DLLAPI Limits
{
  /** Lower joint limits */
  Eigen::VectorXd ql;
  /** Upper joint limits */
  Eigen::VectorXd qu;
  /** Lower joint velocity limits */
  Eigen::VectorXd vl;
  /** Upper joint velocity limits */
  Eigen::VectorXd vu;
  /** Lower joint acceleration limits */
  Eigen::VectorXd al;
  /** Upper joint acceleration limits */
  Eigen::VectorXd au;
  /** Lower joint jerk limits */
  Eigen::VectorXd jl;
  /** Upper joint jerk limits */
  Eigen::VectorXd ju;
  /** Lower joint torque limits */
  Eigen::VectorXd tl;
  /** Upper joint torque limits */
  Eigen::VectorXd tu;
  /** Lower joint torque derivative limits */
  Eigen::VectorXd tdl;
  /** Upper joint torque derivative limits */
  Eigen::VectorXd tdu;
};

} // namespace mc_tvm
