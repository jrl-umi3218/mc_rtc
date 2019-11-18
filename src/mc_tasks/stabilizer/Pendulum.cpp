/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include <mc_tasks/stabilizer/Pendulum.h>

namespace mc_tasks
{
namespace stabilizer
{

Pendulum::Pendulum(const Eigen::Vector3d & gravity,
                   const Eigen::Vector3d & com,
                   const Eigen::Vector3d & comd,
                   const Eigen::Vector3d & comdd)
: gravity_(gravity)
{
  reset(com, comd, comdd);
}

void Pendulum::reset(const Eigen::Vector3d & com, const Eigen::Vector3d & comd, const Eigen::Vector3d & comdd)
{
  constexpr double DEFAULT_HEIGHT = 0.8; // [m]
  double DEFAULT_LAMBDA = gravity_.norm() / DEFAULT_HEIGHT;
  com_ = com;
  comd_ = comd;
  comdd_ = comdd;
  comddd_ = Eigen::Vector3d::Zero();
  omega_ = std::sqrt(DEFAULT_LAMBDA);
  zmp_ = com_ + (gravity_ - comdd_) / DEFAULT_LAMBDA;
  zmpd_ = comd_ - comddd_ / DEFAULT_LAMBDA;
}

void Pendulum::integrateIPM(Eigen::Vector3d zmp, double lambda, double dt)
{
  Eigen::Vector3d com_prev = com_;
  Eigen::Vector3d comd_prev = comd_;
  omega_ = std::sqrt(lambda);
  zmp_ = zmp;

  Eigen::Vector3d vrp = zmp_ - gravity_ / lambda;
  double ch = std::cosh(omega_ * dt);
  double sh = std::sinh(omega_ * dt);
  comdd_ = lambda * (com_prev - zmp_) + gravity_;
  comd_ = comd_prev * ch + omega_ * (com_prev - vrp) * sh;
  com_ = com_prev * ch + comd_prev * sh / omega_ - vrp * (ch - 1.0);

  // default values for third-order terms
  comddd_ = Eigen::Vector3d::Zero();
  zmpd_ = comd_ - comddd_ / lambda;
}

void Pendulum::integrateCoMJerk(const Eigen::Vector3d & comddd, double dt)
{
  com_ += dt * (comd_ + dt * (comdd_ / 2 + dt * (comddd / 6)));
  comd_ += dt * (comdd_ + dt * (comddd / 2));
  comdd_ += dt * comddd;
  comddd_ = comddd;
}

void Pendulum::resetCoMHeight(double height, const Contact & plane)
{
  auto n = plane.normal();
  com_ += (height + n.dot(plane.p() - com_)) * n;
  comd_ -= n.dot(comd_) * n;
  comdd_ -= n.dot(comdd_) * n;
  comddd_ -= n.dot(comddd_) * n;
}

void Pendulum::completeIPM(const Contact & plane)
{
  auto n = plane.normal();
  auto gravitoInertial = gravity_ - comdd_;
  double lambda = n.dot(gravitoInertial) / n.dot(plane.p() - com_);
  zmp_ = com_ + gravitoInertial / lambda;
  zmpd_ = comd_ - comddd_ / lambda;
  omega_ = std::sqrt(lambda);
}

} // namespace stabilizer
} // namespace mc_tasks
