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
#include <mc_tasks/api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/StdVector>

#include <cmath>

namespace mc_tasks
{
namespace lipm_stabilizer
{

using HrepXd = std::pair<Eigen::MatrixXd, Eigen::VectorXd>;

/** Contact state: set of feet in contact.
 *
 */
enum class MC_TASKS_DLLAPI ContactState
{
  Left,
  Right
};

namespace internal
{

/**
 * @brief This class wraps information about contact surfaces used by the stabilizer.
 */
struct MC_TASKS_DLLAPI Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Contact() = default;

  /**
   * @brief Constructs a contact from a surface attached to the ankle frame of a robot
   *
   * @param robot Robot from which to create the contact
   * @param surfaceName Name of surface attached to the robot's ankle. It is
   * assumed that the surface frame is defined as follows:
   * - z axis going from the environment towards the robot
   * - x aligned with the sagital direction
   * - y axis aligned with the lateral direction
   *
   * @param friction sole friction
   */
  Contact(const mc_rbdyn::Robot & robot, const std::string & surfaceName, double friction);

  /**
   * @brief Constructs a Contact with user-specified surface pose
   * @param surfacePose Pose of the contact surface frame
   *
   * \see Contact(const mc_rbdyn::Robot&, const std::string &, double);
   */
  Contact(const mc_rbdyn::Robot & robot,
          const std::string & surfaceName,
          const sva::PTransformd & surfacePose,
          double friction);

  /**
   * @brief Halfspace representation of contact area in world frame.
   *
   * @param vertical Normalized vector parallel to gravity
   */
  HrepXd hrep(const Eigen::Vector3d & vertical) const;

  /** H-representation of contact wrench cones.
   *
   * See <https://hal.archives-ouvertes.fr/hal-02108449/document> for
   * technical details on the derivation of this formula.
   *
   * \note Uses halfLength_, halfWidth_ computed from the surface points in findSurfaceBoundaries(), friction_: sole
   * friction
   */
  const Eigen::Matrix<double, 16, 6> & wrenchFaceMatrix() const
  {
    return wrenchFaceMatrix_;
  }

  /** Sagittal unit vector of the contact frame.
   *
   */
  Eigen::Vector3d sagittal() const
  {
    return surfacePose_.rotation().row(0);
  }

  /** Lateral unit vector of the contact frame.
   *
   */
  Eigen::Vector3d lateral() const
  {
    return surfacePose_.rotation().row(1);
  }

  /** Normal unit vector of the contact frame.
   *
   */
  Eigen::Vector3d normal() const
  {
    return surfacePose_.rotation().row(2);
  }
  /** Shorthand for position.
   *
   */
  const Eigen::Vector3d & position() const
  {
    return surfacePose_.translation();
  }

  double halfWidth() const
  {
    return halfWidth_;
  }

  double halfLength() const
  {
    return halfLength_;
  }

  /**
   * World position of projected ankle frame into surface frame.
   * Orientation is that of the target contact frame.
   */
  const sva::PTransformd & anklePose() const
  {
    return anklePose_;
  }

  Eigen::Vector3d sagital()
  {
    return surfacePose_.rotation().row(0);
  }

  Eigen::Vector3d lateral()
  {
    return surfacePose_.rotation().row(1);
  }

  Eigen::Vector3d vertical()
  {
    return surfacePose_.rotation().row(2);
  }

  const std::string & surfaceName() const
  {
    return surfaceName_;
  }

  const sva::PTransformd & surfacePose() const
  {
    return surfacePose_;
  }

  /**
   * @brief Returns the contact polygon defined by the 4 vertices of the min/max
   * coordinates along the surface's sagital and normal direction
   */
  const std::vector<Eigen::Vector3d> & polygon() const
  {
    return contactPolygon_;
  }

  /**
   * @brief World coordinates of the point furthest back in the surface's
   * sagital direction
   */
  double xmin() const
  {
    return contactPolygon_[0].x();
  }

  /**
   * @brief World coordinates of the point furthest front in the surface's
   * sagital direction
   */
  double xmax() const
  {
    return contactPolygon_[1].x();
  }

  /**
   * @brief World coordinates of the point furthest right in the surface's
   * lateral direction
   */
  double ymin() const
  {
    return contactPolygon_[3].y();
  }

  /**
   * @brief World coordinates of the point furthest left in the surface's
   * lateral direction
   */
  double ymax() const
  {
    return contactPolygon_[0].y();
  }

private:
  /**
   * @brief Finds the surface boundaries in the sagital/lateral plane from surface points, and compute its properties:
   * halfLength_, halfWidth_, and contact polygon.
   */
  void findSurfaceBoundaries(const mc_rbdyn::Surface & surface);

private:
  std::string surfaceName_; /**< Name of the contact surface in robot model. */
  sva::PTransformd anklePose_; /**< Contact frame rooted at the ankle */
  sva::PTransformd surfacePose_; /**< Plücker transform of the contact in the inertial frame. */

  double halfLength_ = 0.; /**< Half-length of the contact rectangle in [m]. */
  double halfWidth_ = 0.; /**< Half-width of the contact rectangle in [m]. */
  double friction_ = 0.7; /**< Friction coefficient for the contact surface */

  Eigen::Matrix<double, 16, 6> wrenchFaceMatrix_; /**< Matrix of single-contact wrench cone inequalities */

  std::vector<Eigen::Vector3d>
      contactPolygon_; /**< Polygon of the surface boundaries along the sagital/lateral plane */
};

} // namespace internal

struct MC_TASKS_DLLAPI ContactDescription
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactState first;
  internal::Contact second;
};

using ContactDescriptionVector = std::vector<ContactDescription, Eigen::aligned_allocator<ContactDescription>>;

} // namespace lipm_stabilizer
} // namespace mc_tasks

namespace mc_rtc
{

template<>
struct ConfigurationLoader<mc_tasks::lipm_stabilizer::ContactState>
{
  static mc_tasks::lipm_stabilizer::ContactState load(const mc_rtc::Configuration & config)
  {
    using ContactState = mc_tasks::lipm_stabilizer::ContactState;
    const std::string & s = config;
    if(s == "Left")
    {
      return ContactState::Left;
    }
    else if(s == "Right")
    {
      return ContactState::Right;
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("ContactState should be one of [Left, Right], {} requested.", s);
    }
  }

  static mc_rtc::Configuration save(const mc_tasks::lipm_stabilizer::ContactState & contact)
  {
    using ContactState = mc_tasks::lipm_stabilizer::ContactState;
    mc_rtc::Configuration config;
    switch(contact)
    {
      case ContactState::Left:
        config = "Left";
        break;
      case ContactState::Right:
        config = "Right";
        break;
    }
    return config;
  }
};

} // namespace mc_rtc
