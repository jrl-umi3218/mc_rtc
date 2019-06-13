#pragma once
#include <mc_tasks/SplineTrajectoryTask.h>

namespace mc_trajectory
{
struct BSpline;
struct InterpolatedRotation;
} // namespace mc_trajectory

namespace mc_tasks
{

struct MC_TASKS_DLLAPI BSplineTrajectoryTask : public SplineTrajectoryTask<BSplineTrajectoryTask>
{
public:
  /**
   * @brief Creates a trajectory that follows a bspline curve
   *
   * @param robots Robots controlled by the task
   * @param robotIndex Which robot is controlled
   * @param surfaceName Surface controlled by the task, should belong to the
   * controlled robot
   * @param duration Duration of motion (eg time it takes to go from the current
   * surface position to the curve's final point)
   * @param stiffness Stiffness of the underlying TrajectoryTask (position and
   * orientation)
   * @param posW Task weight (position)
   * @param oriW Task weight (orientation)
   * @param target Final world pose to reach
   * @param posWp Waypoints in position
   * @param oriWp Waypoints in orientation specified as pairs of (time,
   * orientation). The surface orientation will be interpolated in-between
   * waypoints.
   */
  BSplineTrajectoryTask(const mc_rbdyn::Robots & robots,
                        unsigned int robotIndex,
                        const std::string & surfaceName,
                        double duration,
                        double stiffness,
                        double posW,
                        double oriW,
                        const sva::PTransformd & target,
                        const std::vector<Eigen::Vector3d> & posWp,
                        const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp = {});

  const mc_trajectory::BSpline & spline() const
  {
    return *bspline.get();
  };
  mc_trajectory::BSpline & spline()
  {
    return *bspline.get();
  };

  void target(const sva::PTransformd & target);

  void addToGUI(mc_rtc::gui::StateBuilder & gui);

private:
  /**
   * @brief Control points for the bezier curve (position)
   *
   * @param posWp Vector of position control points for the bezier curve
   */
  void posWaypoints(const std::vector<Eigen::Vector3d> & posWp);

protected:
  std::unique_ptr<mc_trajectory::BSpline> bspline = nullptr;
};

} // namespace mc_tasks
