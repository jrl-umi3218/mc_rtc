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
public:
  InterpolatedRotation(const std::vector<std::pair<double, Eigen::Matrix3d>> & waypoints);
  Eigen::Matrix3d eval(double t);

  std::vector<std::pair<double, Eigen::Matrix3d>> & waypoints();
  void waypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & waypoints);

protected:
  std::vector<double> time_;
  std::vector<std::pair<double, Eigen::Matrix3d>> waypoints_;
};

} // namespace mc_trajectory
