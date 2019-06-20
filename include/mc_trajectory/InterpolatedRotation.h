#pragma once
#include <mc_rtc/GUIState.h>
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
  /*! \brief Evaluate the orientation at time t
   *
   * \param t Time at which the curve should be evaluated
   *
   * \returns Interpolated orientation at time t
   */
  Eigen::Matrix3d eval(double t);

  /*! \brief Defines waypoints in orientation
   *
   * @param waypoints Waypoints specified as pairs of [time, orientation].
   * Shouldn't include starting and target orienation (use start() and target() instead).
   * Time should be 0<time<duration.
   */
  void waypoints(const std::vector<waypoint_t> & waypoints);
  /*! \brief Returns orientation waypoints
   *
   * \returns Orientation waypoints.
   * Doesn't include starting and target orienation (use start() and target() instead).
   */
  std::vector<waypoint_t> & waypoints();

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

  /*! \brief Sets the orientation at time t=0
   *
   * \param ori Desired orientation
   */
  void start(const Eigen::Matrix3d & ori);
  /*! \brief Orientation at time t=0
   *
   * \returns Orientation at t=0
   */
  const Eigen::Matrix3d & start() const;

  /*! \brief Target orientation at time t=duration
   *
   * \param ori Final target orientation
   */
  void target(const Eigen::Matrix3d & ori);
  /*! \brief  Target orientation at time t=duration
   *
   * \returns Final target orientation
   */
  const Eigen::Matrix3d & target() const;

protected:
  double duration_;
  Eigen::Matrix3d start_;
  Eigen::Matrix3d target_;
  std::vector<std::pair<double, Eigen::Matrix3d>> waypoints_;
  bool needsUpdate_ = false;
  std::vector<waypoint_t> all_waypoints_;
};

} // namespace mc_trajectory
