#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_trajectory/BSpline.h>
#include <mc_trajectory/InterpolatedRotation.h>

namespace mc_tasks
{

BSplineTrajectoryTask::BSplineTrajectoryTask(const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             const std::string & surfaceName,
                                             double duration,
                                             double stiffness,
                                             double posW,
                                             double oriW,
                                             const sva::PTransformd & target,
                                             const std::vector<Eigen::Vector3d> & posWp,
                                             const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: SplineTrajectoryTask<BSplineTrajectoryTask>(robots, robotIndex, surfaceName, duration, stiffness, posW, oriW, oriWp),
  bspline(duration)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  type_ = "bspline_trajectory";
  name_ = "bspline_trajectory_" + robot.name() + "_" + surfaceName_;
  posWaypoints(posWp);
  SplineTrajectoryBase::target(target);
}

void BSplineTrajectoryTask::posWaypoints(const std::vector<Eigen::Vector3d> & posWp)
{
  mc_trajectory::BSpline::t_point_t waypoints;
  waypoints.reserve(posWp.size() + 2);
  const auto & robot = robots.robot(rIndex);
  const auto & X_0_s = robot.surface(surfaceName_).X_0_s(robot);
  waypoints.push_back(X_0_s.translation());
  for(const auto & wp : posWp)
  {
    waypoints.push_back(wp);
  }
  waypoints.push_back(finalTarget_.translation());
  bspline.waypoints(waypoints);
}

void BSplineTrajectoryTask::target(const sva::PTransformd & target)
{
  bspline.target(target.translation());
}

Eigen::Vector3d BSplineTrajectoryTask::target() const
{
  return bspline.target();
}

void BSplineTrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  SplineTrajectoryBase::addToGUI(gui);
  bspline.addToGUI(gui, {"Tasks", name_});
}

} // namespace mc_tasks

namespace
{
static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "bspline_trajectory",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      sva::PTransformd finalTarget_;
      std::vector<Eigen::Vector3d> waypoints;
      std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
      const auto robotIndex = config("robotIndex");

      if(config.has("targetSurface"))
      { // Target defined from a target surface, with an offset defined
        // in the surface coordinates
        const auto & c = config("targetSurface");
        const auto & targetSurfaceName = c("surface");
        const auto & robot = solver.robot(c("robotIndex"));

        const sva::PTransformd & targetSurface = robot.surface(targetSurfaceName).X_0_s(robot);
        const Eigen::Vector3d trans = c("translation", Eigen::Vector3d::Zero().eval());
        const Eigen::Matrix3d rot = c("rotation", Eigen::Matrix3d::Identity().eval());
        sva::PTransformd offset(rot, trans);
        finalTarget_ = offset * targetSurface;

        if(c.has("controlPoints"))
        {
          // Control points offsets defined wrt to the target surface frame
          const auto & controlPoints = c("controlPoints");
          waypoints.resize(controlPoints.size());
          for(unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            const Eigen::Vector3d wp = controlPoints[i];
            sva::PTransformd X_offset(wp);
            waypoints[i] = (X_offset * targetSurface).translation();
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
      }
      else
      { // Absolute target pose
        finalTarget_ = config("target");

        if(config.has("controlPoints"))
        {
          // Control points defined in world coordinates
          const auto & controlPoints = config("controlPoints");
          waypoints.resize(controlPoints.size());
          for(unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            const Eigen::Vector3d wp = controlPoints[i];
            waypoints[i] = wp;
          }
        }

        oriWp = config("oriWaypoints", std::vector<std::pair<double, Eigen::Matrix3d>>{});
      }

      std::shared_ptr<mc_tasks::BSplineTrajectoryTask> t = std::make_shared<mc_tasks::BSplineTrajectoryTask>(
          solver.robots(), robotIndex, config("surface"), config("duration"), config("stiffness"), config("posWeight"),
          config("oriWeight"), finalTarget_, waypoints, oriWp);
      t->load(solver, config);
      const auto displaySamples = config("displaySamples", t->displaySamples());
      t->displaySamples(displaySamples);
      return t;
    });
}
