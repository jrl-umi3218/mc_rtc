#pragma once

#include <mc_tasks/SplineTrajectoryTask.h>

namespace mc_tasks
{
template<typename Derived>
SplineTrajectoryTask<Derived>::SplineTrajectoryTask(const mc_rbdyn::Robots & robots,
                                                    unsigned int robotIndex,
                                                    const std::string & surfaceName,
                                                    double duration,
                                                    double stiffness,
                                                    double posW,
                                                    double oriW,
                                                    const sva::PTransformd & target,
                                                    const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: TrajectoryTask(robots, robotIndex, surfaceName, duration, stiffness, posW, oriW)
{
  TrajectoryTask::target(target);
  oriWaypoints(oriWp);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::update()
{
  auto & spline = static_cast<Derived &>(*this).spline();
  // spline.splev + orientation update
  spline.samplingPoints(samples_);

  // Interpolate position
  auto res = spline.splev({t}, 2);
  Eigen::Vector3d & pos = res[0][0];
  Eigen::Vector3d & vel = res[0][1];
  Eigen::Vector3d & acc = res[0][2];

  // Interpolate orientation
  Eigen::Matrix3d ori_target = orientation_spline->eval(t);
  sva::PTransformd target(ori_target, pos);

  // Set the trajectory tracking task targets from the trajectory.
  Eigen::VectorXd refVel(6);
  Eigen::VectorXd refAcc(6);
  refVel.head<3>() = Eigen::Vector3d::Zero();
  refVel.tail<3>() = vel;
  refAcc.head<3>() = Eigen::Vector3d::Zero();
  refAcc.tail<3>() = acc;
  this->refVel(refVel);
  this->refAccel(refAcc);
  this->refPose(target);

  TrajectoryTask::update();
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
{
  const auto & robot = robots.robot(rIndex);
  const auto & X_0_s = robot.surface(surfaceName).X_0_s(robot);
  oriWp_.push_back(std::make_pair(0., X_0_s.rotation()));
  for(const auto & wp : oriWp)
  {
    oriWp_.push_back(wp);
  }
  oriWp_.push_back(std::make_pair(duration, X_0_t.rotation()));
  orientation_spline.reset(new mc_trajectory::InterpolatedRotation(oriWp_));
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  auto & derived = static_cast<Derived &>(*this);
  TrajectoryTask::addToGUI(gui);
  auto & spline = static_cast<Derived &>(*this).spline();
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform("pos", [this]() {
                   return robots.robot(rIndex).surface(surfaceName).X_0_s(robots.robot(rIndex));
                 }));

  // XXX would be nice to implement a different style for rotation element, currently this is quite confusing
  // to see which is an orientation and which is a bspline control point.
  for(unsigned i = 1; i < orientation_spline->waypoints().size(); ++i)
  {
    gui.addElement({"Tasks", name_, "Orientation Control Points"},
                   mc_rtc::gui::Rotation("control_point_ori_" + std::to_string(i),
                                         [this, i, &spline]() {
                                           const auto & wp = orientation_spline->waypoints()[i];

                                           // Get position of orientation waypoint along the spline
                                           const auto & res = spline.splev({wp.first}, 2);
                                           const Eigen::Vector3d pos = res[0][0];
                                           return sva::PTransformd(wp.second, pos);
                                         },
                                         [this, i](const Eigen::Quaterniond & ori) {
                                           auto & wp = orientation_spline->waypoints()[i];
                                           wp.second = ori.toRotationMatrix();
                                         }));
  }
  derived.addToGUI(gui);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  auto & derived = static_cast<Derived &>(*this);
  TrajectoryTask::removeFromGUI(gui);
  gui.removeCategory({"Tasks", name_, "Orientation Control Points"});
  derived.removeFromGUI(gui);
}

} // namespace mc_tasks
