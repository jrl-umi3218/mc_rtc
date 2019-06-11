#pragma once

#include <mc_tasks/TrajectoryTask.h>
#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_tasks
{
template<typename Derived>
struct SplineTrajectoryTask : public TrajectoryTask
{
  using SplineTrajectoryBase = SplineTrajectoryTask<Derived>;

  // Constructor handles task creation + orientation waypoints if any
  SplineTrajectoryTask(const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       const std::string & surfaceName,
                       double duration,
                       double stiffness,
                       double posW,
                       double oriW,
                       const sva::PTransformd & target,
                       const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);

  void update() override;
  void oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);
  void addToGUI(mc_rtc::gui::StateBuilder & gui);
  void removeFromGUI(mc_rtc::gui::StateBuilder & gui);

  // Factorize as much stuff as possible, basically this class can take care of everything except the spline
protected:
  std::shared_ptr<mc_trajectory::InterpolatedRotation> orientation_spline;
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp_;
};
} // namespace mc_tasks

#include <mc_tasks/SplineTrajectoryTask.hpp>
