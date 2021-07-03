/*
 * Copyright (c) 2009,
 * @author Mitsuharu Morisawa
 *
 * AIST
 *
 * All rights reserved.
 *
 * This program is made available under the terms of the Eclipse Public License
 * v1.0 which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 */
#pragma once
#include <mc_planning/api.h>
#include <Eigen/Core>

namespace mc_planning
{
namespace linear_control_system
{

// linear control for siso system
struct MC_PLANNING_DLLAPI LinearControl3
{
  LinearControl3();
  void Initialize();
  virtual void update();
  virtual void update(const Eigen::Vector3d & x_ref_);
  virtual void update(const double dt);
  virtual void update(const Eigen::Vector3d & x_ref_, const double dt);
  void setReference(const Eigen::Vector3d & x_ref_)
  {
    x_ref = x_ref_;
  };
  void setReference(double x1, double x2, double x3)
  {
    x_ref << x1, x2, x3;
  };

protected:
  // system matrices and feedback gain
  Eigen::Matrix3d A;
  Eigen::Vector3d B;
  Eigen::RowVector3d C;
  Eigen::Vector3d K;
  Eigen::Vector3d x; ///< state variable
  Eigen::Vector3d x_ref; ///< reference state variable
  double y; ///< output
  double u; ///< input
};
} // namespace linear_control_system
} // namespace mc_planning
