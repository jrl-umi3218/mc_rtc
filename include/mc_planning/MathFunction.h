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
#include <Eigen/Geometry>

namespace mc_planning
{

const double epsilonAngle = 1e-16;

template<typename T>
inline T Max(const T & a, const T & b)
{
  return ((a) > (b) ? (a) : (b));
}

template<typename T>
inline T Min(const T & a, const T & b)
{
  return ((a) < (b) ? (a) : (b));
}

template<typename T>
inline int Sign(const T & x)
{
  return ((x < 0) ? (-1) : ((x > 0) ? 1 : 0));
}

template<typename T>
inline T Pow2(const T & a)
{
  return dot((a), (a));
}

template<typename T>
inline T polynomial3(const T & t)
{
  return ((3.0 - 2.0 * (t)) * (t) * (t));
}

template<typename T>
inline T dpolynomial3(const T & t)
{
  return ((6.0 - 6.0 * (t)) * (t));
}

template<typename T>
inline T ddpolynomial3(const T & t)
{
  return (6.0 - 12.0 * (t));
}

template<typename T>
inline T polynomial5(const T & t)
{
  return (((6.0 * (t)-15.0) * (t) + 10.0) * (t) * (t) * (t));
}

template<typename T>
inline T dpolynomial5(const T & t)
{
  return (((30.0 * (t)-60.0) * (t) + 30.0) * (t) * (t));
}

template<typename T>
inline T ddpolynomial5(const T & t)
{
  return (((120.0 * (t)-180.0) * (t) + 60.0) * (t));
}

#if defined(__GNUC__)
#  define __ATTRIBUTE_ALWAYS_INLINE__ __attribute__((always_inline))
#else
#  define __ATTRIBUTE_ALWAYS_INLINE__
#endif

inline Eigen::Matrix3d __ATTRIBUTE_ALWAYS_INLINE__ rotFromRoll(const double & roll)
{
  const double c = cos(roll);
  const double s = sin(roll);
  Eigen::Matrix3d r;
  r << 1.0, 0, 0, 0, c, -s, 0, s, c;
  return r;
}

inline Eigen::Matrix3d __ATTRIBUTE_ALWAYS_INLINE__ rotFromPitch(const double & pitch)
{
  const double c = cos(pitch);
  const double s = sin(pitch);
  Eigen::Matrix3d r;
  r << c, 0, s, 0, 1.0, 0, -s, 0, c;
  return r;
}

inline Eigen::Matrix3d __ATTRIBUTE_ALWAYS_INLINE__ rotFromYaw(const double & yaw)
{
  const double c = cos(yaw);
  const double s = sin(yaw);
  Eigen::Matrix3d r;
  r << c, -s, 0, s, c, 0, 0, 0, 1.0;
  return r;
}

inline Eigen::Matrix3d __ATTRIBUTE_ALWAYS_INLINE__ rotFromYrp(const double & y, const double & r, const double & p)
{
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()))
      .toRotationMatrix();
}

inline Eigen::Matrix3d __ATTRIBUTE_ALWAYS_INLINE__ rotFromAngleAxis(const double & angles, const Eigen::Vector3d & axis)
{
  return Eigen::AngleAxisd(angles, axis).toRotationMatrix();
  // return rodrigues(axis, angles);
}

inline Eigen::Matrix3d __ATTRIBUTE_ALWAYS_INLINE__ rotFromAngleAxis(const Eigen::Vector3d & omega,
                                                                    const double eps = 1.0e-6)
{
#if 0
    Eigen::Vector3d w(omega);
    for( int axis = 0 ; axis <= 2 ; axis++ ){
      while( w(axis) >  M_PI ) w(axis) -= M_PI;
      while( w(axis) < -M_PI ) w(axis) += M_PI;
    }
    double th = w.norm();
    return (th > eps) ? Eigen::AngleAxisd(th, w.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity();
#else
  double th = omega.norm();
  return (th > eps) ? Eigen::AngleAxisd(th, omega.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity();
  // return (th > eps) ? rodrigues(omega.normalized(), th) : Eigen::Matrix3d::Identity();
#endif
}

inline Eigen::Matrix3d __ATTRIBUTE_ALWAYS_INLINE__ rotFromAngleAxis(double e1,
                                                                    double e2,
                                                                    double e3,
                                                                    const double eps = 1.0e-6)
{
  return rotFromAngleAxis(Eigen::Vector3d(e1, e2, e3), eps);
}

/// !!! yrpFromRot can't check singularity (p = pi/2)
Eigen::Vector3d yrpFromRot(const Eigen::Matrix3d & m);
double yawFromRot(const Eigen::Matrix3d & r);
Eigen::Vector3d omegaFromRotApproximation(const Eigen::Matrix3d & r);

inline Eigen::Matrix3d mergeTiltWithYaw(const Eigen::Vector3d & Rtez, const Eigen::Matrix3d & R2)
{

  Eigen::Matrix3d R_temp1, R_temp2;
  const Eigen::Vector3d & ez = Eigen::Vector3d::UnitZ();
  const Eigen::Vector3d & v1 = Rtez;
  Eigen::Vector3d mlxv1 = (R2.transpose() * Eigen::Vector3d::UnitX()).cross(v1);
  double n2 = mlxv1.squaredNorm();

  if(n2 > epsilonAngle * epsilonAngle)
  {
    R_temp1 << -Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitX(), ez;
    mlxv1 /= sqrt(n2);
    R_temp2 << mlxv1.transpose(), v1.cross(mlxv1).transpose(), v1.transpose();
    return R_temp1 * R_temp2;
  }
  else
  {
    mlxv1 = (R2.transpose() * Eigen::Vector3d::UnitY()).cross(v1).normalized();
    R_temp2 << mlxv1.transpose(), v1.cross(mlxv1).transpose(), v1.transpose();
    return R_temp2.transpose();
  }
}

inline Eigen::Matrix3d mergeRoll1Pitch1WithYaw2(const Eigen::Matrix3d & R1, const Eigen::Matrix3d & R2)
{
  return mergeTiltWithYaw(R1.transpose() * Eigen::Vector3d::UnitZ(), R2);
}

double Saturation(const double & data, const double ulimit, const double llimit);
double Threshold(const double & data, const double ulimit, const double llimit);
double NormalizedTrapezoidCurve(int n_now, int n_ini, int n_acc, int n_dec, int n_end);

template<class T>
T LowPassFilter(const T & data, T & data_lpf, const double gain)
{
  data_lpf += (data - data_lpf) * gain;
  return data_lpf;
}

template<class T>
T HighPassFilter(const T & data, T & data_hpf, const double gain)
{
  data_hpf += (data - data_hpf * (1.0 - gain));
  return data_hpf;
}

} // namespace mc_planning
