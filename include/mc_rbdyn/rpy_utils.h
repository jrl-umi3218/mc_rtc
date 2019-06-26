/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

/** Rotation matrix from roll-pitch-yaw angles.
 *
 * \param r Roll angle.
 *
 * \param p Pitch angle.
 *
 * \param y Yaw angle.
 *
 * \returns E Rotation matrix.
 *
 */
inline Eigen::Matrix3d rpyToMat(const double & r, const double & p, const double & y)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  return sva::RotX(r) * sva::RotY(p) * sva::RotZ(y);
#pragma GCC diagnostic pop
}

/** Rotation matrix from roll-pitch-yaw angles.
 *
 * \param rpy Vector of roll-pitch-yaw angles.
 *
 * \returns E Rotation matrix.
 *
 */
inline Eigen::Matrix3d rpyToMat(const Eigen::Vector3d & rpy)
{
  return rpyToMat(rpy(0), rpy(1), rpy(2));
}

/** Plucker transfrom from roll-pitch-yaw angles.
 *
 * \param rpy Vector of roll-pitch-yaw angles.
 *
 * \returns X Plucker transform with no translation.
 *
 */
inline sva::PTransformd rpyToPT(const Eigen::Vector3d & rpy)
{
  return sva::PTransformd(rpyToMat(rpy(0), rpy(1), rpy(2)));
}

/** Plucker transfrom from roll-pitch-yaw angles.
 *
 * \param r Roll angle.
 *
 * \param p Pitch angle.
 *
 * \param y Yaw angle.
 *
 * \returns X Plucker transform with no translation.
 *
 */
inline sva::PTransformd rpyToPT(const double & r, const double & p, const double & y)
{
  return sva::PTransformd(rpyToMat(r, p, y));
}

/** Roll-pitch-yaw from rotation matrix.
 *
 * \param R Rotation matrix.
 *
 * \returns rpy Vector of roll-pitch-yaw angles (Euler sequence (0, 1, 2)).
 *
 * Adapted from formula (289) of "Representing attitude: Euler angles, unit
 * quaternions, and rotation vectors".
 *
 */
inline Eigen::Vector3d rpyFromMat(const Eigen::Matrix3d & E)
{
  double roll = atan2(E(1, 2), E(2, 2));
  double pitch = -asin(E(0, 2));
  double yaw = atan2(E(0, 1), E(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

/** Roll-pitch-yaw from unit quaternion.
 *
 * \param quat Input unit quaternion.
 *
 * \returns rpy Vector of roll-pitch-yaw angles (Euler sequence (0, 1, 2)).
 *
 * Be careful that Eigen's conversion to rotation matrices is the OPPOSITE of
 * that from "Representing attitude: Euler angles, unit quaternions, and
 * rotation vectors", i.e. we need to transpose/flip signs from Equation (290)
 * of this paper.
 *
 */
inline Eigen::Vector3d rpyFromQuat(const Eigen::Quaterniond & quat)
{
  return rpyFromMat(quat.toRotationMatrix());
}

} // namespace mc_rbdyn
