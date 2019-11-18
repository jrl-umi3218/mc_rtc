/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <cmath>

namespace mc_tasks
{
namespace stabilizer
{
using HrepXd = std::pair<Eigen::MatrixXd, Eigen::VectorXd>;

/** Contact state: set of feet in contact.
 *
 */
enum class ContactState
{
  DoubleSupport,
  LeftFoot,
  RightFoot
};

/** Contacts wrap foot frames with extra info from the footstep plan.
 *
 */
struct Contact
{
  /** Empty constructor.
   *
   * Leaves the contact's Plücker transform uninitialized.
   *
   */
  Contact() {}

  /** Define from Plücker transform.
   *
   * \param pose Plücker transform from inertial to contact frame.
   *
   */
  Contact(const sva::PTransformd & pose) : pose(pose) {}

  /** Sagittal unit vector of the contact frame.
   *
   */
  Eigen::Vector3d sagittal() const
  {
    return pose.rotation().row(0);
  }

  /** Lateral unit vector of the contact frame.
   *
   */
  Eigen::Vector3d lateral() const
  {
    return pose.rotation().row(1);
  }

  /** Normal unit vector of the contact frame.
   *
   */
  Eigen::Vector3d normal() const
  {
    return pose.rotation().row(2);
  }

  /** World position of the contact frame.
   *
   */
  const Eigen::Vector3d & position() const
  {
    return pose.translation();
  }

  /** Shorthand for position.
   *
   */
  const Eigen::Vector3d & p() const
  {
    return position();
  }

  /** Position of ankle from foot center frame.
   *
   */
  Eigen::Vector3d anklePos() const
  {
    if(surfaceName == "LeftFootCenter")
    {
      return p() - 0.015 * sagittal() - 0.01 * lateral();
    }
    else if(surfaceName == "RightFootCenter")
    {
      return p() - 0.015 * sagittal() + 0.01 * lateral();
    }
    else
    {
      LOG_ERROR("Cannot compute anklePos for surface " << surfaceName);
      return p();
    }
  }

  /** Get frame rooted at the ankle.
   *
   */
  sva::PTransformd anklePose() const
  {
    return {pose.rotation(), anklePos()};
  }

  /** Shorthand for world x-coordinate.
   *
   */
  double x() const
  {
    return position()(0);
  }

  /** Shorthand for world y-coordinate.
   *
   */
  double y() const
  {
    return position()(1);
  }

  /** Shorthand for world z-coordinate.
   *
   */
  double z() const
  {
    return position()(2);
  }

  /** Corner vertex of the contact area.
   *
   */
  Eigen::Vector3d vertex0() const
  {
    return position() + halfLength * sagittal() + halfWidth * lateral();
  }

  /** Corner vertex of the contact area.
   *
   */
  Eigen::Vector3d vertex1() const
  {
    return position() + halfLength * sagittal() - halfWidth * lateral();
  }

  /** Corner vertex of the contact area.
   *
   */
  Eigen::Vector3d vertex2() const
  {
    return position() - halfLength * sagittal() - halfWidth * lateral();
  }

  /** Corner vertex of the contact area.
   *
   */
  Eigen::Vector3d vertex3() const
  {
    return position() - halfLength * sagittal() + halfWidth * lateral();
  }

  /** Minimum coordinate for vertices of the contact area.
   *
   */
  template<int i>
  double minCoord() const
  {
    return std::min(std::min(vertex0()(i), vertex1()(i)), std::min(vertex2()(i), vertex3()(i)));
  }

  /** Maximum coordinate for vertices of the contact area.
   *
   */
  template<int i>
  double maxCoord() const
  {
    return std::max(std::max(vertex0()(i), vertex1()(i)), std::max(vertex2()(i), vertex3()(i)));
  }

  /** Minimum world x-coordinate of the contact area.
   *
   */
  double xmin() const
  {
    return minCoord<0>();
  }

  /** Maximum world x-coordinate of the contact area.
   *
   */
  double xmax() const
  {
    return maxCoord<0>();
  }

  /** Minimum world y-coordinate of the contact area.
   *
   */
  double ymin() const
  {
    return minCoord<1>();
  }

  /** Maximum world y-coordinate of the contact area.
   *
   */
  double ymax() const
  {
    return maxCoord<1>();
  }

  /** Minimum world z-coordinate of the contact area.
   *
   */
  double zmin() const
  {
    return minCoord<2>();
  }

  /** Maximum world z-coordinate of the contact area.
   *
   */
  double zmax() const
  {
    return maxCoord<2>();
  }

  /**
   * @brief Halfspace representation of contact area in world frame.
   *
   * @param vertical Normalized vector parallel to gravity
   */
  HrepXd hrep(const Eigen::Vector3d & vertical) const
  {
    Eigen::Matrix<double, 4, 2> localHrepMat, worldHrepMat;
    Eigen::Matrix<double, 4, 1> localHrepVec, worldHrepVec;
    localHrepMat << +1, 0, -1, 0, 0, +1, 0, -1;
    localHrepVec << halfLength, halfLength, halfWidth, halfWidth;
    if((normal() - vertical).norm() > 1e-3)
    {
      LOG_WARNING("Contact is not horizontal");
    }
    const sva::PTransformd & X_0_c = pose;
    worldHrepMat = localHrepMat * X_0_c.rotation().topLeftCorner<2, 2>();
    worldHrepVec = worldHrepMat * X_0_c.translation().head<2>() + localHrepVec;
    return HrepXd(worldHrepMat, worldHrepVec);
  }

  /** Move contact by a given magnitude in a random direction.
   *
   * \param magnitude Absolute displacement after noising.
   *
   */
  Contact addNoise(double magnitude) const
  {
    Contact noisedContact = *this;
    Eigen::Vector3d unitRandom = Eigen::Vector3d::Random().normalized();
    Eigen::Vector3d displacement = magnitude * unitRandom;
    noisedContact.pose = sva::PTransformd{displacement} * this->pose;
    return noisedContact;
  }

  /** Compute floating base transform that puts the robot in contact.
   *
   * \param robot Robot model (including its multi-body configuration).
   *
   */
  inline sva::PTransformd robotTransform(const mc_rbdyn::Robot & robot) const
  {
    const sva::PTransformd & X_0_c = pose;
    const sva::PTransformd & X_0_fb = robot.posW();
    sva::PTransformd X_s_0 = robot.surfacePose(surfaceName).inv();
    sva::PTransformd X_s_fb = X_0_fb * X_s_0;
    return X_s_fb * X_0_c;
  }

public:
  Eigen::Vector3d refVel =
      Eigen::Vector3d::Zero(); /**< Desired CoM velocity when the robot is supporting itself on this contact. */
  double halfLength = 0.; /**< Half-length of the contact rectangle in [m]. */
  double halfWidth = 0.; /**< Half-width of the contact rectangle in [m]. */
  mc_rtc::Configuration
      swingConfig; /**< Additional configuration for swing foot trajectories that originate from this contact. */
  std::string surfaceName = ""; /**< Name of the contact surface in robot model. */
  sva::PTransformd pose; /**< Plücker transform of the contact in the inertial frame. */
  unsigned id = 0; /**< Index of contact in footstep plan. */
};

/** Apply Plucker transform to contact frame.
 *
 * \param X Transform to apply.
 *
 * \param contact Contact frame.
 *
 */
inline Contact operator*(const sva::PTransformd & X, const Contact & contact)
{
  Contact result = contact;
  result.pose = X * contact.pose;
  return result;
}
} // namespace stabilizer
} // namespace mc_tasks

namespace mc_rtc
{
template<>
struct ConfigurationLoader<mc_tasks::stabilizer::Contact>
{
  static mc_tasks::stabilizer::Contact load(const mc_rtc::Configuration & config)
  {
    mc_tasks::stabilizer::Contact contact;
    contact.pose = config("pose");
    config("half_length", contact.halfLength);
    config("half_width", contact.halfWidth);
    config("ref_vel", contact.refVel);
    config("surface", contact.surfaceName);
    if(config.has("swing"))
    {
      contact.swingConfig = config("swing");
    }
    return contact;
  }

  static mc_rtc::Configuration save(const mc_tasks::stabilizer::Contact & contact)
  {
    mc_rtc::Configuration config;
    config.add("half_length", contact.halfLength);
    config.add("half_width", contact.halfWidth);
    config.add("pose", contact.pose);
    config.add("ref_vel", contact.refVel);
    config.add("surface", contact.surfaceName);
    if(!contact.swingConfig.empty())
    {
      config("swing") = contact.swingConfig;
    }
    return config;
  }
};
} // namespace mc_rtc
