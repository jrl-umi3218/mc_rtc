/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/ExactCubicTrajectoryTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_trajectory/ExactCubic.h>
#include <mc_trajectory/InterpolatedRotation.h>

#include <mc_rtc/gui/Arrow.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

ExactCubicTrajectoryTask::ExactCubicTrajectoryTask(const mc_rbdyn::Robots & robots,
                                                   unsigned int robotIndex,
                                                   const std::string & surfaceName,
                                                   double duration,
                                                   double stiffness,
                                                   double weight,
                                                   const sva::PTransformd & target,
                                                   const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                                                   const Eigen::Vector3d & init_vel,
                                                   const Eigen::Vector3d & init_acc,
                                                   const Eigen::Vector3d & end_vel,
                                                   const Eigen::Vector3d & end_acc,
                                                   const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: ExactCubicTrajectoryTask(robots.robot(robotIndex).frame(surfaceName),
                           duration,
                           stiffness,
                           weight,
                           target,
                           posWp,
                           init_vel,
                           init_acc,
                           end_vel,
                           end_acc,
                           oriWp)
{
}

ExactCubicTrajectoryTask::ExactCubicTrajectoryTask(const mc_rbdyn::RobotFrame & frame,
                                                   double duration,
                                                   double stiffness,
                                                   double weight,
                                                   const sva::PTransformd & target,
                                                   const std::vector<std::pair<double, Eigen::Vector3d>> & posWp,
                                                   const Eigen::Vector3d & init_vel,
                                                   const Eigen::Vector3d & init_acc,
                                                   const Eigen::Vector3d & end_vel,
                                                   const Eigen::Vector3d & end_acc,
                                                   const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: SplineTrajectoryTask<ExactCubicTrajectoryTask>(frame, duration, stiffness, weight, target.rotation(), oriWp),
  bspline(duration, frame.position().translation(), target.translation(), posWp, init_vel, init_acc, end_vel, end_acc)
{
  type_ = "exact_cubic_trajectory";
  name_ = "exact_cubic_trajectory_" + frame.robot().name() + "_" + frame.name();
  bspline.constraints(init_vel, init_acc, end_vel, end_acc);
  initialPose_ = frame_->position();
}

void ExactCubicTrajectoryTask::posWaypoints(const std::vector<std::pair<double, Eigen::Vector3d>> & posWp)
{
  bspline.waypoints(posWp);
}

void ExactCubicTrajectoryTask::constraints(const Eigen::Vector3d & init_vel,
                                           const Eigen::Vector3d & init_acc,
                                           const Eigen::Vector3d & end_vel,
                                           const Eigen::Vector3d & end_acc)
{
  bspline.constraints(init_vel, init_acc, end_vel, end_acc);
}

void ExactCubicTrajectoryTask::targetPos(const Eigen::Vector3d & target)
{
  bspline.target(target);
}

const Eigen::Vector3d & ExactCubicTrajectoryTask::targetPos() const
{
  return bspline.target();
}

void ExactCubicTrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  SplineTrajectoryBase::addToGUI(gui);

  bspline.addToGUI(gui, {"Tasks", name_});

  gui.addElement(
      {"Tasks", name_, "Velocity Constraints"},
      mc_rtc::gui::Arrow(
          "Initial", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 1., 1.)),
          [this]() -> Eigen::Vector3d { return initialPose_.translation(); },
          [this]() -> Eigen::Vector3d { return initialPose_.translation() + bspline.init_vel(); }),
      mc_rtc::gui::Arrow(
          "Final", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 1., 1.)),
          [this]() -> Eigen::Vector3d { return SplineTrajectoryBase::target().translation(); },
          [this]() -> Eigen::Vector3d { return SplineTrajectoryBase::target().translation() + bspline.end_vel(); }));
}

} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "exact_cubic_trajectory",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      sva::PTransformd finalTarget_;
      std::vector<std::pair<double, Eigen::Vector3d>> waypoints;
      std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "exact_cubic_trajectory");
      Eigen::Vector3d init_vel, end_vel, init_acc, end_acc;

      bool has_targetSurface = config.has("targetSurface");
      bool has_targetFrame = config.has("targetFrame");

      if(has_targetSurface || has_targetFrame)
      { // Target defined from a target surface, with an offset defined
        // in the surface coordinates
        if(has_targetSurface)
        {
          mc_rtc::log::deprecated("ExactCubicTrajectoryTaskLoading", "targetSurface", "targetFrame");
        }
        const auto & c = config(has_targetSurface ? "targetSurface" : "targetFrame");
        const auto & targetSurfaceName = c(has_targetSurface ? "surface" : "frame");
        const auto & robot = robotFromConfig(c, solver.robots(), "exact_cubic_trajectory::targetFrame");

        const sva::PTransformd & targetSurface = robot.surface(targetSurfaceName).X_0_s(robot);
        const Eigen::Vector3d trans = c("translation", Eigen::Vector3d::Zero().eval());
        const Eigen::Matrix3d rot = c("rotation", Eigen::Matrix3d::Identity().eval());
        sva::PTransformd offset(rot, trans);
        finalTarget_ = offset * targetSurface;

        if(c.has("controlPoints"))
        {
          // Control points offsets defined wrt to the target surface frame
          std::vector<std::pair<double, Eigen::Vector3d>> controlPoints = c("controlPoints");
          waypoints.resize(controlPoints.size());
          for(unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            const Eigen::Vector3d wp = controlPoints[i].second;
            sva::PTransformd X_offset(wp);
            waypoints[i].first = controlPoints[i].first;
            waypoints[i].second = (X_offset * targetSurface).translation();
          }
        }

        if(c.has("oriWaypoints"))
        {
          std::vector<std::pair<double, Eigen::Matrix3d>> oriWaypoints = c("oriWaypoints");
          for(const auto & wp : oriWaypoints)
          {
            const sva::PTransformd offset{wp.second};
            const sva::PTransformd ori = offset * targetSurface;
            oriWp.push_back(std::make_pair(wp.first, ori.rotation()));
          }
        }
        init_vel = targetSurface.rotation().inverse() * c("init_vel", Eigen::Vector3d::Zero().eval());
        init_acc = targetSurface.rotation().inverse() * c("init_acc", Eigen::Vector3d::Zero().eval());
        end_vel = targetSurface.rotation().inverse() * c("end_vel", Eigen::Vector3d::Zero().eval());
        end_acc = targetSurface.rotation().inverse() * c("end_acc", Eigen::Vector3d::Zero().eval());
      }
      else
      { // Absolute target pose
        finalTarget_ = config("target");

        if(config.has("controlPoints"))
        {
          // Control points defined in world coordinates
          std::vector<std::pair<double, Eigen::Vector3d>> controlPoints = config("controlPoints");
          waypoints.resize(controlPoints.size());
          for(unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            waypoints[i] = controlPoints[i];
          }
        }
        init_vel = config("init_vel", Eigen::Vector3d::Zero().eval());
        init_acc = config("init_acc", Eigen::Vector3d::Zero().eval());
        end_vel = config("end_vel", Eigen::Vector3d::Zero().eval());
        end_acc = config("end_acc", Eigen::Vector3d::Zero().eval());
        oriWp = config("oriWaypoints", std::vector<std::pair<double, Eigen::Matrix3d>>{});
      }

      const auto & frame = [&]() -> const mc_rbdyn::RobotFrame & {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("ExactCubicTrajectoryTask", "surface", "frame");
          return solver.robots().robot(robotIndex).frame(config("surface"));
        }
        return solver.robots().robot(robotIndex).frame(config("frame"));
      }();
      auto t = std::make_shared<mc_tasks::ExactCubicTrajectoryTask>(
          frame, config("duration", 10.0), config("stiffness", 100.0), config("weight", 500.0), finalTarget_, waypoints,
          init_vel, init_acc, end_vel, end_acc, oriWp);
      t->load(solver, config);
      const auto displaySamples = config("displaySamples", t->displaySamples());
      t->displaySamples(displaySamples);
      t->pause(config("paused", false));
      return t;
    });
}
