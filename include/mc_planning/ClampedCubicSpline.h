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
#include <mc_planning/CubicSplineBase.h>

namespace mc_planning
{
namespace motion_interpolator
{

template<typename T>
class ClampedCubicSpline : public CubicSplineBase<T>
{
protected:
  void setSplineCoeff(unsigned int n_base, unsigned int n_segment, unsigned int n_offset);

public:
  ClampedCubicSpline(const double & scale = 1.0, const double & eps = 1.0e-3);
};

template<typename T>
ClampedCubicSpline<T>::ClampedCubicSpline(const double & scale, const double & eps)
: CubicSplineBase<T>(scale, eps, CLAMPED_CUBIC_SPLINE)
{
}

template<typename T>
void ClampedCubicSpline<T>::setSplineCoeff(unsigned int n_base, unsigned int n_size, unsigned int n_offset)
{
  using namespace motion_interpolator;

  typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin() + n_offset;

  const double & data_dot_ini = iter->vel;
  const double & data_dot_end = (iter + n_size - 1)->vel;

  if(n_size <= 2)
  {
    CubicSplineBase<T>::coeff[n_base].resize(n_size * 2, 0);
    return;
  }

  CubicSplineBase<T>::coeff[n_base].resize(n_size * 2);
  CubicSplineBase<T>::coeff[n_base][0] = -0.5;
  CubicSplineBase<T>::coeff[n_base][n_size] =
      (3.0 / ((double)((iter + 1)->t - iter->t) * InterpolatorBase<T>::scale))
      * (((iter + 1)->pos - iter->pos) / ((double)((iter + 1)->t - iter->t) * InterpolatorBase<T>::scale)
         - data_dot_ini);
  for(unsigned int i = 1; i < n_size; i++)
  {
    CubicSplineBase<T>::coeff[n_base][i] = 0.0;
    CubicSplineBase<T>::coeff[n_base][i + n_size] = 0.0;
  }
  const double qn = 0.5;
  const double un =
      (3.0 / ((double)((iter + n_size - 1)->t - (iter + n_size - 2)->t) * InterpolatorBase<T>::scale))
      * (data_dot_end
         - ((iter + n_size - 1)->pos - (iter + n_size - 2)->pos)
               / ((double)((iter + n_size - 1)->t - (iter + n_size - 2)->t) * InterpolatorBase<T>::scale));

  CubicSplineBase<T>::makeSplineTable(n_base, n_size, n_offset, qn, un);
}
} // end of namespace motion_interpolator
} // namespace mc_planning
