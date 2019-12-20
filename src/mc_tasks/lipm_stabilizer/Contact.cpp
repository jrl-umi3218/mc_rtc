/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include <mc_tasks/lipm_stabilizer/Contact.h>

namespace mc_tasks
{
namespace lipm_stabilizer
{

namespace internal
{

Contact::Contact(const mc_rbdyn::Robot & robot, const std::string & surfaceName)
: Contact(robot, surfaceName, robot.surfacePose(surfaceName))
{
}

Contact::Contact(const mc_rbdyn::Robot & robot, const std::string & surfaceName, const sva::PTransformd & surfacePose)
: surfaceName_(surfaceName)
{
  const auto & surface = robot.surface(surfaceName);
  if(surface.type() != "planar")
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "LIPMStabilizer contact expects a planar surface attached to the robot's ankle. Surface "
                            << surfaceName << " with type " << surface.type() << " not supported");
  }

  surfacePose_ = surfacePose;

  // Compute anke pos
  Eigen::Vector3d t_s_b = surface.X_b_s().inv().translation();
  double sagitalProj = t_s_b.dot(sagital());
  double lateralProj = t_s_b.dot(lateral());
  anklePose_.translation() = surfacePose_.translation() + sagitalProj * sagital() + lateralProj * lateral();
  anklePose_.rotation() = surfacePose_.rotation();

  findSurfaceBoundaries(surface);
}

void Contact::findSurfaceBoundaries(const mc_rbdyn::Surface & surface)
{
  const auto & surfacePoints = surface.points();
  // Find boundaries in surface frame along the surface's sagital (x) and lateral (y) direction
  double minSagital = std::numeric_limits<double>::max();
  double minLateral = std::numeric_limits<double>::max();
  double maxSagital = -std::numeric_limits<double>::max();
  double maxLateral = -std::numeric_limits<double>::max();
  for(const auto & point : surfacePoints)
  {
    double x = point.translation().x();
    double y = point.translation().y();
    minSagital = std::min(minSagital, x);
    maxSagital = std::max(maxSagital, x);
    minLateral = std::min(minLateral, y);
    maxLateral = std::max(maxLateral, y);
  }

  halfLength_ = (maxSagital - minSagital) / 2.;
  halfWidth_ = (maxLateral - minLateral) / 2.;

  // Add world points to the support polygon
  contactPolygon_.push_back(surfacePose_.translation()
                            + surfacePose_.rotation().inverse() * Eigen::Vector3d{minSagital, maxLateral, 0.});
  contactPolygon_.push_back(surfacePose_.translation()
                            + surfacePose_.rotation().inverse() * Eigen::Vector3d{maxSagital, maxLateral, 0.});
  contactPolygon_.push_back(surfacePose_.translation()
                            + surfacePose_.rotation().inverse() * Eigen::Vector3d{maxSagital, minLateral, 0.});
  contactPolygon_.push_back(surfacePose_.translation()
                            + surfacePose_.rotation().inverse() * Eigen::Vector3d{minSagital, minLateral, 0.});
}

HrepXd Contact::hrep(const Eigen::Vector3d & vertical) const
{
  Eigen::Matrix<double, 4, 2> localHrepMat, worldHrepMat;
  Eigen::Matrix<double, 4, 1> localHrepVec, worldHrepVec;
  localHrepMat << +1, 0, -1, 0, 0, +1, 0, -1;
  localHrepVec << halfLength_, halfLength_, halfWidth_, halfWidth_;
  if((normal() - vertical).norm() > 1e-3)
  {
    LOG_WARNING("Contact is not horizontal");
  }
  const sva::PTransformd & X_0_c = surfacePose_;
  worldHrepMat = localHrepMat * X_0_c.rotation().topLeftCorner<2, 2>();
  worldHrepVec = worldHrepMat * X_0_c.translation().head<2>() + localHrepVec;
  return HrepXd(worldHrepMat, worldHrepVec);
}

} // namespace internal
} // namespace lipm_stabilizer
} // namespace mc_tasks
