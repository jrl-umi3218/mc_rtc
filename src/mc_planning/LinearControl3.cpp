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
#include <mc_planning/LinearControl3.h>

namespace mc_planning
{

namespace linear_control_system
{

LinearControl3::LinearControl3()
{
  LinearControl3::Initialize();
}

void LinearControl3::Initialize()
{
  A.setZero();
  B.setZero();
  C.setZero();
  K.setZero();
  x.setZero();
  x_ref.setZero();
  y = 0.0;
  u = 0.0;
}

void LinearControl3::update()
{
  u = -K.dot(x_ref - x);
  x = A * x + B * u;
  y = C.dot(x);
}

void LinearControl3::update(const Eigen::Vector3d & x_ref_)
{
  x_ref = x_ref_;
  update();
}

void LinearControl3::update(const double dt)
{
  u = -K.dot(x_ref - x);
  Eigen::Vector3d dx(A * x + B * u);
  x += dx * dt;
  y = C.dot(x);
}

void LinearControl3::update(const Eigen::Vector3d & x_ref_, const double dt)
{
  x_ref = x_ref_;
  update(dt);
}

} // namespace linear_control_system
} // namespace mc_planning
