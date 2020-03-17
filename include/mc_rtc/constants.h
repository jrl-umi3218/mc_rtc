#pragma once
#include <Eigen/Core>

#ifndef EIGEN_PI
#  define EIGEN_PI 3.141592653589793238462643383279502884197169399375105820974944592307816406L
#endif

namespace mc_rtc
{
namespace constants
{

constexpr double GRAVITY = 9.80665; // ISO 80000-3

/**< Gravity positive along the vertical axis */
const Eigen::Vector3d gravity = Eigen::Vector3d{0., 0., GRAVITY};
const Eigen::Vector3d vertical = Eigen::Vector3d{0., 0., 1.};
constexpr double PI = static_cast<double>(EIGEN_PI);

/**
 * @brief Converts degrees to radians
 *
 * @param degrees Angle in degrees
 *
 * @return Angle in radians
 */
constexpr double toDeg(const double degrees)
{
  return degrees * 180. / PI;
}

/**
 * @brief Converts radians to degrees
 *
 * @param rad Angle in radians
 *
 * @return Angle in degrees
 */
constexpr double toRad(const double rad)
{
  return rad * PI / 180.;
}

} // namespace constants
} // namespace mc_rtc
