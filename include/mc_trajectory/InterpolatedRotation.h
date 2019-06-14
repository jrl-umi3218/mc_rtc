#pragma once
#include <mc_rtc/GUIState.h>
#include <mc_trajectory/api.h>

#include <Eigen/Geometry>
#include <utility>
#include <vector>

namespace mc_trajectory
{

/**
 * @brief Interpolates the rotation between waypoints
 */
struct MC_TRAJECTORY_DLLAPI InterpolatedRotation
{
  using waypoint_t = std::pair<double, Eigen::Matrix3d>;

public:
  InterpolatedRotation();
  InterpolatedRotation(const std::vector<waypoint_t> & waypoints);
  Eigen::Matrix3d eval(double t);

  void waypoints(const std::vector<waypoint_t> & waypoints);
  std::vector<waypoint_t> & waypoints();

  void waypoint(size_t idx, const Eigen::Matrix3d & ori);
  const waypoint_t & waypoint(size_t idx) const;

  void target(const Eigen::Matrix3d & ori);
  const Eigen::Matrix3d & target() const;

protected:
  std::vector<double> time_;
  std::vector<std::pair<double, Eigen::Matrix3d>> waypoints_;
};

} // namespace mc_trajectory
