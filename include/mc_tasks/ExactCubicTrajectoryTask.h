#pragma once
#include <mc_tasks/TrajectoryTask.h>

namespace mc_trajectory
{
struct ExactCubicTrajectory;
struct InterpolatedRotation;
} // namespace mc_trajectory

namespace mc_tasks
{

struct MC_TASKS_DLLAPI ExactCubicTrajectoryTask : public TrajectoryTask
{
public:
  ExactCubicTrajectoryTask(const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           const std::string & surfaceName,
                           double duration,
                           double stiffness,
                           double posW,
                           double oriW,
                           const sva::PTransformd & target,
                           const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                           const Eigen::Vector3d & init_vel,
                           const Eigen::Vector3d & init_acc,
                           const Eigen::Vector3d & end_vel,
                           const Eigen::Vector3d & end_acc,
                           const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);

  void update() override;

private:
  void posWaypoints(const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                    const Eigen::Vector3d & init_vel,
                    const Eigen::Vector3d & init_acc,
                    const Eigen::Vector3d & end_vel,
                    const Eigen::Vector3d & end_acc);
  void oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp);

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void removeFromGUI(mc_rtc::gui::StateBuilder &) override;

protected:
  std::shared_ptr<mc_trajectory::ExactCubicTrajectory> bspline = nullptr;
  std::shared_ptr<mc_trajectory::InterpolatedRotation> orientation_spline = nullptr;
  std::vector<std::pair<double, Eigen::Matrix3d>> oriWp_;
};

} // namespace mc_tasks
