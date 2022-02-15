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

Contact::Contact(const mc_rbdyn::Robot & robot, const std::string & surfaceName, double friction)
: Contact(robot, surfaceName, robot.surfacePose(surfaceName), friction)
{
}

Contact::Contact(const mc_rbdyn::Robot & robot,
                 const std::string & surfaceName,
                 const sva::PTransformd & surfacePose,
                 const double friction)
: surfaceName_(surfaceName), friction_(friction)
{
  const auto & surface = robot.surface(surfaceName);
  if(surface.type() != "planar")
  {
    mc_rtc::log::error_and_throw(
        "LIPMStabilizer contact expects a planar surface attached to the robot's ankle. Surface {} "
        "with type {} not supported",
        surfaceName, surface.type());
  }

  surfacePose_ = surfacePose;

  // Compute anke pos
  Eigen::Vector3d t_s_b = surface.X_b_s().inv().translation();
  double sagitalProj = t_s_b.dot(sagital());
  double lateralProj = t_s_b.dot(lateral());
  anklePose_.translation() = surfacePose_.translation() + sagitalProj * sagital() + lateralProj * lateral();
  anklePose_.rotation() = surfacePose_.rotation();

  findSurfaceBoundaries(surface);

  double X = halfLength_;
  double Y = halfWidth_;
  double mu = friction_;
  // clang-format off
  wrenchFaceMatrix_ <<
    // mx,  my,  mz,  fx,  fy,            fz,
        0,   0,   0,  -1,   0,           -mu,
        0,   0,   0,  +1,   0,           -mu,
        0,   0,   0,   0,  -1,           -mu,
        0,   0,   0,   0,  +1,           -mu,
       -1,   0,   0,   0,   0,            -Y,
       +1,   0,   0,   0,   0,            -Y,
        0,  -1,   0,   0,   0,            -X,
        0,  +1,   0,   0,   0,            -X,
      +mu, +mu,  -1,  -Y,  -X, -(X + Y) * mu,
      +mu, -mu,  -1,  -Y,  +X, -(X + Y) * mu,
      -mu, +mu,  -1,  +Y,  -X, -(X + Y) * mu,
      -mu, -mu,  -1,  +Y,  +X, -(X + Y) * mu,
      +mu, +mu,  +1,  +Y,  +X, -(X + Y) * mu,
      +mu, -mu,  +1,  +Y,  -X, -(X + Y) * mu,
      -mu, +mu,  +1,  -Y,  +X, -(X + Y) * mu,
      -mu, -mu,  +1,  -Y,  -X, -(X + Y) * mu;
  // clang-format on
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
    // Points are defined in body frame, convert to surface frame
    Eigen::Vector3d surfacePoint = surface.X_b_s().rotation() * (point.translation() - surface.X_b_s().translation());
    double x = surfacePoint.x();
    double y = surfacePoint.y();
    minSagital = std::min(minSagital, x);
    maxSagital = std::max(maxSagital, x);
    minLateral = std::min(minLateral, y);
    maxLateral = std::max(maxLateral, y);
  }

  halfLength_ = (maxSagital - minSagital) / 2.;
  halfWidth_ = (maxLateral - minLateral) / 2.;

  if(maxSagital + minSagital > 1e-4 || maxLateral + minLateral > 1e-4)
  {
    mc_rtc::log::warning("LIPMStabilizer contact surface is expected to be a centered rectangle, but the surface {} is "
                         "not centered (sagital: {},{}, lateral: {}, {})",
                         surface.name(), minSagital, maxSagital, minLateral, maxLateral);
  }

  // Add world points to the support polygon
  contactPolygon_.push_back(surfacePose_.translation()
                            + surfacePose_.rotation().inverse() * Eigen::Vector3d{minSagital, maxLateral, 0.});
  contactPolygon_.push_back(surfacePose_.translation()
                            + surfacePose_.rotation().inverse() * Eigen::Vector3d{maxSagital, maxLateral, 0.});
  contactPolygon_.push_back(surfacePose_.translation()
                            + surfacePose_.rotation().inverse() * Eigen::Vector3d{maxSagital, minLateral, 0.});
  contactPolygon_.push_back(surfacePose_.translation()
                            + surfacePose_.rotation().inverse() * Eigen::Vector3d{minSagital, minLateral, 0.});

  const auto & xMinMax =
      std::minmax_element(contactPolygon_.begin(), contactPolygon_.end(),
                          [](const Eigen::Vector3d & v1, const Eigen::Vector3d & v2) { return v1.x() < v2.x(); });
  const auto & yMinMax =
      std::minmax_element(contactPolygon_.begin(), contactPolygon_.end(),
                          [](const Eigen::Vector3d & v1, const Eigen::Vector3d & v2) { return v1.y() < v2.y(); });
  xyMin_ << xMinMax.first->x(), yMinMax.first->y();
  xyMax_ << xMinMax.second->x(), yMinMax.second->y();
}

HrepXd Contact::hrep(const Eigen::Vector3d & vertical) const
{
  Eigen::Matrix<double, 4, 2> localHrepMat, worldHrepMat;
  Eigen::Matrix<double, 4, 1> localHrepVec, worldHrepVec;
  localHrepMat << +1, 0, -1, 0, 0, +1, 0, -1;
  localHrepVec << halfLength_, halfLength_, halfWidth_, halfWidth_;
  if((normal() - vertical).norm() > 1e-3)
  {
    mc_rtc::log::warning("Contact is not horizontal");
    ;
  }
  const sva::PTransformd & X_0_c = surfacePose_;
  worldHrepMat = localHrepMat * X_0_c.rotation().topLeftCorner<2, 2>();
  worldHrepVec = worldHrepMat * X_0_c.translation().head<2>() + localHrepVec;
  return HrepXd(worldHrepMat, worldHrepVec);
}

} // namespace internal
} // namespace lipm_stabilizer
} // namespace mc_tasks
