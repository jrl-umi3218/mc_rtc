#pragma once
#include <mc_tasks/TrajectoryTask.h>

namespace mc_trajectory
{
struct ExactCubic;
struct InterpolatedRotation;
} // namespace mc_trajectory

namespace mc_tasks
{

struct MC_TASKS_DLLAPI ExactCubicTrajectoryTask : public TrajectoryTask
{
public:
  /**
   * @brief Trajectory following an exact cubic spline with given initial and
   * final acceleration/velocity. The curve will pass exactly through the
   * specified position waypoints (see posWaypoints) and will interpolate
   * between orientation waypoints. Initial and final acceleration/velocity will
   * also be enforced.
   *
   * @param robots Robots controlled by the task
   * @param robotIndex  Which robot is controlled
   * @param surfaceName Surface controlled by the task, should belong to the
   * controlled robot
   * @param duration Duration of motion (eg time it takes to go from the current
   * surface position to the curve's final point)
   * @param stiffness Task stiffness (position and orientation), see TrajectoryTask
   * @param posW Task weight (position), see TrajectoryTask
   * @param oriW Task weight (orientation), see TrajectoryTask
   * @param target Final world pose to reach
   * @param posWp Waypoints in position specified as pairs of [time, position]
   * @param init_vel Initial velocity of the curve (default: Zero)
   * @param init_acc Initial acceleration of the curve (default: Zero)
   * @param end_vel Final velocity of the curve (default: Zero)
   * @param enc_acc Final acceleration of the curve (default: Zero)
   * @param oriWp Waypoints in orientation, specified as pairs of [time,
   * orientation]. Orientation is interpolated in between waypoints. (default :
   * none)
   */
  ExactCubicTrajectoryTask(const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           const std::string & surfaceName,
                           double duration,
                           double stiffness,
                           double posW,
                           double oriW,
                           const sva::PTransformd & target,
                           const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                           const Eigen::Vector3d & init_vel = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & init_acc = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & end_vel = Eigen::Vector3d::Zero(),
                           const Eigen::Vector3d & end_acc = Eigen::Vector3d::Zero(),
                           const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

  void update() override;

private:
  /**
   * @brief Waypoints in position. The curve will pass exactly through these
   * waypoints. Initial and final velocities and accelerations can be specified.
   *
   * @param posWp Waypoints in position, specified as pairs of [time, position]
   * @param init_vel Initial velocity of the curve (default: Zero)
   * @param init_acc Initial acceleration of the curve (default: Zero)
   * @param end_vel Final velocity of the curve (default: Zero)
   * @param end_acc Final acceleration of the curve (default: Zero)
   */
  void posWaypoints(const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                    const Eigen::Vector3d & init_vel = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d & init_acc = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d & end_vel = Eigen::Vector3d::Zero(),
                    const Eigen::Vector3d & end_acc = Eigen::Vector3d::Zero());
  /**
   * @brief Waypoints in orientation. The orientation will be interpolated in
   * between waypoints.
   *
   * @param oriWp Waypoints in orientation specified as pairs of
   * [time, orientation]
   */
  void oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;

protected:
  std::shared_ptr<mc_trajectory::ExactCubic> bspline = nullptr;
  std::shared_ptr<mc_trajectory::InterpolatedRotation> orientation_spline = nullptr;
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp_;
};

} // namespace mc_tasks
