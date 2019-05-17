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
: TrajectoryTask(robots, robotIndex, surfaceName, duration, stiffness, posW, oriW)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  type_ = "bspline_trajectory";
  name_ = "bspline_trajectory_" + robot.name() + "_" + surfaceName;
  this->target(target);
  posWaypoints(posWp);
  oriWaypoints(oriWp);
}

void BSplineTrajectoryTask::posWaypoints(const std::vector<Eigen::Vector3d> & posWp)
{
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.reserve(posWp.size() + 2);
  const auto & robot = robots.robot(rIndex);
  const auto & X_0_s = robot.surface(surfaceName).X_0_s(robot);
  waypoints.push_back(X_0_s.translation());
  for(const auto & wp : posWp)
  {
    waypoints.push_back(wp);
  }
  waypoints.push_back(X_0_t.translation());
  bspline.reset(new mc_trajectory::BSpline(waypoints, duration));
}

void BSplineTrajectoryTask::oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
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

void BSplineTrajectoryTask::update()
{
  bspline->samplingPoints(samples_);

  // Interpolate position
  auto res = bspline->splev({t}, 2);
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
  this->refAcc(refAcc);
  this->refPose(target);

  TrajectoryTask::update();
}

void BSplineTrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTask::addToGUI(gui);
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform("pos", [this]() {
                   return robots.robot(rIndex).surface(surfaceName).X_0_s(robots.robot(rIndex));
                 }));

  bspline->addToGUI(gui, {"Tasks", name_, "Position Control Points"});

  // XXX would be nice to implement a different style for rotation element, currently this is quite confusing
  // to see which is an orientation and which is a bspline control point.
  for(unsigned i = 1; i < orientation_spline->waypoints().size(); ++i)
  {
    gui.addElement({"Tasks", name_, "Orientation Control Points"},
                   mc_rtc::gui::Rotation("control_point_ori_" + std::to_string(i),
                                         [this, i]() {
                                           const auto & wp = orientation_spline->waypoints()[i];

                                           // Get position of orientation waypoint along the spline
                                           const auto & res = bspline->splev({wp.first}, 2);
                                           const Eigen::Vector3d pos = res[0][0];
                                           return sva::PTransformd(wp.second, pos);
                                         },
                                         [this, i](const Eigen::Quaterniond & ori) {
                                           auto & wp = orientation_spline->waypoints()[i];
                                           wp.second = ori.toRotationMatrix();
                                         }));
  }
}

void BSplineTrajectoryTask::removeFromGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTask::removeFromGUI(gui);
  gui.removeCategory({"Tasks", name_, "Orientation Control Points"});
  gui.removeCategory({"Tasks", name_, "Position Control Points"});
}

} // namespace mc_tasks

namespace
{
static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "bspline_trajectory",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      sva::PTransformd X_0_t;
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
        const Eigen::Vector3d trans = c("offset_translation", Eigen::Vector3d::Zero().eval());
        const Eigen::Matrix3d rot = c("offset_rotation", Eigen::Matrix3d::Identity().eval());
        sva::PTransformd offset(rot, trans);
        X_0_t = offset * targetSurface;

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
        X_0_t = config("target");

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
          config("oriWeight"), X_0_t, waypoints, oriWp);
      t->load(solver, config);
      const auto displaySamples = config("displaySamples", t->displaySamples());
      t->displaySamples(displaySamples);
      return t;
    });
}
