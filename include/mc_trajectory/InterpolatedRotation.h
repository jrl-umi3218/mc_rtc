/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/GUIState.h>
#include <mc_trajectory/Spline.h>
#include <mc_trajectory/api.h>

#include <Eigen/Geometry>
#include <utility>
#include <vector>

namespace mc_trajectory
{

/**
 * \brief Describes a trajectory with smoothly interpolate rotation between waypoints
 */
struct MC_TRAJECTORY_DLLAPI InterpolatedRotation
: Spline<Eigen::Matrix3d, std::vector<std::pair<double, Eigen::Matrix3d>>>
{
  using waypoint_t = std::pair<double, Eigen::Matrix3d>;

public:
  /*! \brief Creates a trajectory that smoothly interpolates rotation in-between
   * waypoints
   *
   * \param duration Duration of the curve
   * \param start Starting orientation at t=0
   * \param target Final orientation at t=duration
   * \param waypoints Optional waypoints as pairs of [time, orientation].
   * Time should be 0<time<duration.
   */
  InterpolatedRotation(double duration,
                       const Eigen::Matrix3d & start,
                       const Eigen::Matrix3d & target,
                       const std::vector<waypoint_t> & waypoints = {});

  void update() override;

  /*! \brief Evaluate the orientation at time t
   *
   * \param t Time at which the curve should be evaluated
   *
   * \returns Interpolated orientation at time t
   */
  Eigen::Matrix3d eval(double t);

  /*! \brief Modifies an existing waypoint
   *
   * @param idx Id of the waypoint
   * @param ori Desired orientation for that waypoint
   */
  void waypoint(size_t idx, const Eigen::Matrix3d & ori);
  /*! \brief Gets an existing waypoint
   *
   * \param idx Id of the waypoint
   *
   * \returns Waypoint with index idx
   */
  const waypoint_t & waypoint(size_t idx) const;

protected:
  std::vector<waypoint_t> all_waypoints_;
};

} // namespace mc_trajectory
