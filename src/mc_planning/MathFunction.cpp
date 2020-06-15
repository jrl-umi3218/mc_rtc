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
#include <mc_planning/MathFunction.h>

namespace mc_planning
{

Eigen::Vector3d yrpFromRot(const Eigen::Matrix3d & m)
{
  double roll = atan2(-m(1, 2), m(2, 2));
  double cr = cos(roll);
  double pitch = atan2(cr * m(0, 2), m(2, 2));
  double cp = cos(pitch), sr = sin(roll);
  double yaw = atan2(cp * (cr * m(1, 0) + sr * m(2, 0)), m(0, 0));

  return Eigen::Vector3d(roll, pitch, yaw);
}

double yawFromRot(const Eigen::Matrix3d & r)
{
  return atan2(r(1, 0), r(0, 0));
}

Eigen::Vector3d omegaFromRotApproximation(const Eigen::Matrix3d & r)
{
  const double k = -0.5;
  return Eigen::Vector3d((r(1, 2) - r(2, 1)) * k, (r(2, 0) - r(0, 2)) * k, (r(0, 1) - r(1, 0)) * k);
}

double Saturation(const double & data, const double ulimit, const double llimit)
{
  if(data > ulimit)
    return ulimit;
  else if(data < llimit)
    return llimit;
  return data;
}

double Threshold(const double & data, const double ulimit, const double llimit)
{
  if(data > ulimit)
    return data - ulimit;
  else if(data < llimit)
    return data - llimit;
  return 0.0;
}

double NormalizedTrapezoidCurve(int n_now, int n_ini, int n_acc, int n_dec, int n_end)
{
  if(n_now < n_ini)
    return 0.0;
  else if(n_now <= n_acc)
    return polynomial3((double)((n_now - n_ini)) / (double)((n_acc - n_ini)));
  else if(n_now <= n_dec)
    return 1.0;
  else if(n_now <= n_end)
    return polynomial3((double)((n_end - n_now)) / (double)((n_end - n_dec)));

  return 0.0;
}

} // namespace mc_planning
