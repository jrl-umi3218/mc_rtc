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
#include "InterpolatorBase.h"
#include <math.h>
#include <vector>

namespace mc_planning
{
namespace motion_interpolator
{

template<class T>
class CubicSplineBase : public InterpolatorBase<T>
{
protected:
  std::vector<std::vector<double>> coeff;

  void getPointToPoint(T t_, double & p, double & v, double & a, unsigned int n_segment);

  virtual void makeSplineTable(unsigned int n_base,
                               unsigned int n_segment,
                               unsigned int n_offset,
                               const double & qn,
                               const double & un);

  virtual void setSplineCoeff(unsigned int n_base, unsigned int n_segment, unsigned int n_offset) = 0;

public:
  CubicSplineBase(const double & scale = 1.0, const double & eps = 1.0e-3, interpolator_type ip_type_ = NOT_SELECTED);

  void get(T t_, double & p, double & v, double & a);

  void update(void);

  void update(T t_);

  void update(T t_from, T t_to);
};

template<class T>
CubicSplineBase<T>::CubicSplineBase(const double & scale, const double & eps, interpolator_type ip_type_)
: InterpolatorBase<T>(scale, eps, ip_type_)
{
}

template<typename T>
void CubicSplineBase<T>::getPointToPoint(T t_, double & p, double & v, double & a, unsigned int n_segment)
{
  using namespace motion_interpolator;

  typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin() + n_segment;

  const T ti = iter->t;
  const T tf = (iter + 1)->t;
  const double t1 = (double)(tf - ti) * InterpolatorBase<T>::scale;

  if(t1 < InterpolatorBase<T>::eps)
  {
    p = iter->pos;
    v = iter->vel;
    a = iter->acc;
  }
  else
  {
    const double t2 = t1 * t1;
    const double t3 = t1 * t2;
    const double t4 = t1 * t3;
    const double t5 = t1 * t4;

    const double & pi = iter->pos;
    const double & vi = iter->vel;
    const double & ai = iter->acc;
    const double & pf = (iter + 1)->pos;
    const double & vf = (iter + 1)->vel;
    const double & af = (iter + 1)->acc;

    const double & c0 = pi;
    const double & c1 = vi;
    const double c2 = ai / 2.0;
    const double c3 = (20.0 * (pf - pi) - (8.0 * vf + 12.0 * vi) * t1 + (af - 3.0 * ai) * t2) / (2.0 * t3);
    const double c4 = (-30.0 * (pf - pi) + (14.0 * vf + 16.0 * vi) * t1 - (2.0 * af - 3.0 * ai) * t2) / (2.0 * t4);
    const double c5 = (12.0 * (pf - pi) - 6.0 * (vf + vi) * t1 + (af - ai) * t2) / (2.0 * t5);

    const double tn = (double)(t_ - ti) * InterpolatorBase<T>::scale;
    p = c0 + (c1 + (c2 + (c3 + (c4 + c5 * tn) * tn) * tn) * tn) * tn;
    v = c1 + (2.0 * c2 + (3.0 * c3 + (4.0 * c4 + 5.0 * c5 * tn) * tn) * tn) * tn;
    a = 2.0 * c2 + (6.0 * c3 + (12.0 * c4 + 20.0 * c5 * tn) * tn) * tn;
  }
}

template<typename T>
void CubicSplineBase<T>::get(T t_, double & p, double & v, double & a)
{
  using namespace motion_interpolator;

  if(InterpolatorBase<T>::data.empty()) return;

  if(InterpolatorBase<T>::data.size() == 1)
  {
    p = InterpolatorBase<T>::data.front().pos;
    v = 0.0;
    a = 0.0;
    return;
  }

  unsigned int n_base = 0;
  unsigned int n_segment_coeff = 0;
  unsigned int n_segment_data = 0;
  typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin();
  if(t_ <= iter->t)
  {
    t_ = iter->t;
  }
  else if(t_ > InterpolatorBase<T>::data.back().t)
  {
    n_base = coeff.size() - 1;
    n_segment_coeff = coeff[n_base].size() / 2 - 2;
    n_segment_data = InterpolatorBase<T>::data.size() - 2;
    iter = InterpolatorBase<T>::data.end() - 2;
    t_ = InterpolatorBase<T>::data.back().t;
  }
  else
  {
    while(t_ > (iter + 1)->t)
    {
      if((iter + 1)->boundary)
      {
        n_base++;
        n_segment_coeff = 0;
      }
      else
        n_segment_coeff++;

      n_segment_data++;
      iter++;
    }
  }

  if(coeff[n_base].size() <= 4)
  {
    getPointToPoint(t_, p, v, a, n_segment_data);
  }
  else
  {
    double h = (double)((iter + 1)->t - iter->t) * InterpolatorBase<T>::scale;
    if(h <= 0.0)
      throw "Bad input to routine splint";
    else
    {
      double dhi = (double)((iter + 1)->t - t_) * InterpolatorBase<T>::scale / h;
      double dlo = (double)(t_ - iter->t) * InterpolatorBase<T>::scale / h;

      p = dhi * iter->pos + dlo * (iter + 1)->pos
          + ((dhi * dhi * dhi - dhi) * coeff[n_base][n_segment_coeff]
             + (dlo * dlo * dlo - dlo) * coeff[n_base][n_segment_coeff + 1])
                * (h * h) / 6.0;
      v = (-iter->pos + (iter + 1)->pos) / h
          + ((-3.0 * dhi * dhi + 1.0) * coeff[n_base][n_segment_coeff]
             + (3.0 * dlo * dlo - 1.0) * coeff[n_base][n_segment_coeff + 1])
                * h / 6.0;
      a = dhi * coeff[n_base][n_segment_coeff] + dlo * coeff[n_base][n_segment_coeff + 1];
    }
  }
}

template<class T>
void CubicSplineBase<T>::makeSplineTable(unsigned int n_base,
                                         unsigned int n_size,
                                         unsigned int n_offset,
                                         const double & qn,
                                         const double & un)
{
  using namespace motion_interpolator;

  typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin() + n_offset;
  for(unsigned int i = 1; i < n_size - 1; i++)
  {
    double sig = ((double)((iter + i)->t - (iter + i - 1)->t)) / ((double)((iter + i + 1)->t - (iter + i - 1)->t));
    double p = sig * coeff[n_base][i - 1] + 2.0;
    coeff[n_base][i] = (sig - 1.0) / p;
    coeff[n_base][i + n_size] = ((iter + i + 1)->pos - (iter + i)->pos)
                                    / ((double)((iter + i + 1)->t - (iter + i)->t) * InterpolatorBase<T>::scale)
                                - ((iter + i)->pos - (iter + i - 1)->pos)
                                      / ((double)((iter + i)->t - (iter + i - 1)->t) * InterpolatorBase<T>::scale);
    coeff[n_base][i + n_size] = (6.0 * coeff[n_base][i + n_size]
                                     / ((double)((iter + i + 1)->t - (iter + i - 1)->t) * InterpolatorBase<T>::scale)
                                 - sig * coeff[n_base][i - 1 + n_size])
                                / p;
  }

  coeff[n_base][n_size - 1] = (un - qn * coeff[n_base][n_size - 2 + n_size]) / (qn * coeff[n_base][n_size - 2] + 1.0);
  for(int j = n_size - 2; j >= 0; j--)
    coeff[n_base][j] = coeff[n_base][j] * coeff[n_base][j + 1] + coeff[n_base][j + n_size];
}

template<typename T>
void CubicSplineBase<T>::update(void)
{
  using namespace motion_interpolator;

  unsigned int n_base_max = 1;
  for(typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin() + 1;
      iter != InterpolatorBase<T>::data.end() - 1; iter++)
  {
    if(iter->boundary) n_base_max++;
  }
  if(coeff.size() < n_base_max) coeff.resize(n_base_max);

  unsigned int n_base = 0;
  unsigned int n_size = 2;
  unsigned int n_data = 2;
  for(typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin() + 1;
      iter != InterpolatorBase<T>::data.end(); iter++)
  {
    if(iter->boundary || (iter == InterpolatorBase<T>::data.end() - 1))
    {
      setSplineCoeff(n_base, n_size, n_data - n_size);
      n_base++;
      n_size = 2;
    }
    else
      n_size++;

    n_data++;
  }
}

template<typename T>
void CubicSplineBase<T>::update(T t_)
{
  using namespace motion_interpolator;

  unsigned int n_base_max = 1;
  for(typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin() + 1;
      iter != InterpolatorBase<T>::data.end() - 1; iter++)
  {
    if(iter->boundary) n_base_max++;
  }
  if(coeff.size() < n_base_max) coeff.resize(n_base_max);

  unsigned int n_base = 0;
  unsigned int n_size = 0;
  unsigned int n_data = 0;
  typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin();
  while((iter + 2 != InterpolatorBase<T>::data.end()) && ((t_ > (iter + 1)->t) || !(iter + 1)->boundary))
  {
    if((iter + 1)->boundary)
    {
      n_base++;
      n_size = 0;
    }
    else
      n_size++;

    n_data++;
    iter++;
  }

  setSplineCoeff(n_base, n_size + 2, n_data - n_size);
}

template<typename T>
void CubicSplineBase<T>::update(T t_from, T t_to)
{
  using namespace motion_interpolator;

  unsigned int n_base_max = 1;
  for(typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin() + 1;
      iter != InterpolatorBase<T>::data.end() - 1; iter++)
  {
    if(iter->boundary) n_base_max++;
  }
  if(coeff.size() < n_base_max) coeff.resize(n_base_max);

  unsigned int n_base = 0;
  unsigned int n_size = 0;
  unsigned int n_data = 0;
  typename std::deque<InterpolatorDataType<T>>::const_iterator iter = InterpolatorBase<T>::data.begin();
  while((iter + 2 != InterpolatorBase<T>::data.end()) && ((t_from > (iter + 1)->t) || !(iter + 1)->boundary))
  {
    if((iter + 1)->boundary)
    {
      n_base++;
      n_size = 0;
    }
    else
      n_size++;

    n_data++;
    iter++;
  }

  setSplineCoeff(n_base, n_size + 2, n_data - n_size);

  unsigned int n_base_to = n_base;
  unsigned int n_size_to = n_size;
  unsigned int n_data_to = n_data;
  iter = InterpolatorBase<T>::data.begin() + n_data;
  while((iter + 2 != InterpolatorBase<T>::data.end()) && ((t_to > (iter + 1)->t) || !(iter + 1)->boundary))
  {
    if((iter + 1)->boundary)
    {
      if(n_base_to != n_base) setSplineCoeff(n_base_to, n_size_to + 2, n_data_to - n_size_to);
      n_base_to++;
      n_size_to = 0;
    }
    else
      n_size_to++;

    n_data_to++;
    iter++;
  }

  if(n_base_to != n_base) setSplineCoeff(n_base_to, n_size_to + 2, n_data_to - n_size_to);
}
} // namespace motion_interpolator
} // namespace mc_planning
