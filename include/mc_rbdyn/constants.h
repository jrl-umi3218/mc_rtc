#pragma once
#include <Eigen/Core>

namespace mc_rbdyn
{
namespace constants
{

constexpr double GRAVITY = 9.80665; // ISO 80000-3

/**< Gravity positive along the vertical axis */
const Eigen::Vector3d gravity = Eigen::Vector3d{0., 0., GRAVITY};
const Eigen::Vector3d vertical = Eigen::Vector3d{0., 0., 1.};

} // namespace constants
} // namespace mc_rbdyn
