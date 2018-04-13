#include <mc_tasks/TrajectoryTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/Surface.h>
#include <mc_trajectory/spline_utils.h>

namespace mc_tasks
{
TrajectoryTask::TrajectoryTask(
    const mc_rbdyn::Robots& robots, unsigned int robotIndex,
    const std::string& surfaceName, const sva::PTransformd& X_0_t,
    double duration, double timeStep, double stiffness, double posW,
    double oriW, const Eigen::MatrixXd& waypoints,
    const std::vector<double> oriWpTime, const std::vector<Eigen::Matrix3d>& oriWp,
    unsigned int nrWP)
    : robots(robots),
      rIndex(robotIndex),
      surfaceName(surfaceName),
      X_0_t(X_0_t),
      wp(waypoints),
      oriWpTime_(oriWpTime),
      oriWp_(oriWp),
      duration(duration),
      timeStep(timeStep),
      t(0.)
{
  const mc_rbdyn::Robot& robot = robots.robot(robotIndex);
  const auto& surface = robot.surface(surfaceName);
  type_ = "trajectory";
  name_ = "trajectory_" + robot.name() + "_" + surface.name();
  X_0_start = surface.X_0_s(robot);

  // Orientation waypoints
  X_0_oriStart = X_0_start.rotation();
  oriTargetWpIndex = 0;
  oriStartTime = 0;
  assert(oriWpTime_.size() == oriWp_.size());
  // Add a virtual waypoint at the end of the trajectory to avoid special-cases
  // in update()
  oriWpTime_.push_back(duration);
  oriWp_.push_back(X_0_t.rotation());

  if(nrWP > 0)
  {
    Eigen::Vector3d start = X_0_start.translation();
    Eigen::Vector3d end = X_0_t.translation();
    wp = mc_trajectory::generateInterpolatedWaypoints(start, end, nrWP);
  }

  transTask.reset(new tasks::qp::TransformTask(robots.mbs(), static_cast<int>(robotIndex), surface.bodyName(), X_0_start, surface.X_b_s()));
  transTrajTask.reset(new tasks::qp::TrajectoryTask(robots.mbs(), static_cast<int>(robotIndex), transTask.get(), stiffness, 2*sqrt(stiffness), 1.0));
  posWeight(posW);
  oriWeight(oriW);

  generateBS();
}

void TrajectoryTask::stiffness(double s)
{
  setGains(s, 2*std::sqrt(s));
}

void TrajectoryTask::damping(double d)
{
  setGains(stiffness(), d);
}

void TrajectoryTask::setGains(double s, double d)
{
  transTrajTask->setGains(s, d);
}

double TrajectoryTask::stiffness() const
{
  return transTrajTask->stiffness();
}

double TrajectoryTask::damping() const
{
  return transTrajTask->damping();
}


void TrajectoryTask::posWeight(const double posWeight)
{
  auto dimWeight = transTrajTask->dimWeight();
  for(unsigned int i = 3; i < 6; ++i)
  {
    dimWeight(i) = posWeight;
  }
  transTrajTask->dimWeight(dimWeight);
}

double TrajectoryTask::posWeight() const
{
  return transTrajTask->dimWeight()(3);
}

void TrajectoryTask::oriWeight(const double oriWeight)
{
  auto dimWeight = transTrajTask->dimWeight();
  for(unsigned int i = 0; i < 3; ++i)
  {
    dimWeight(i) = oriWeight;
  }
  transTrajTask->dimWeight(dimWeight);
}

double TrajectoryTask::oriWeight() const
{
  return transTrajTask->dimWeight()(0);
}

void TrajectoryTask::dimWeight(const Eigen::VectorXd & dimW)
{
  assert(dimW.size() == 6);
  transTrajTask->dimWeight(dimW);
}

Eigen::VectorXd TrajectoryTask::dimWeight() const
{
  return transTrajTask->dimWeight();
}


void TrajectoryTask::target(const sva::PTransformd& target)
{
  X_0_t = target;
  oriWp_[oriWp_.size()-1] = X_0_t.rotation();
  generateBS();
}

sva::PTransformd TrajectoryTask::target() const
{
  return X_0_t;
}

std::vector<Eigen::Vector3d> TrajectoryTask::controlPoints()
{
  std::vector<Eigen::Vector3d> res;
  res.reserve(static_cast<unsigned int>(wp.size()) + 2);
  res.push_back(X_0_start.translation());
  for(int i = 0; i < wp.cols(); ++i)
  {
    Eigen::Vector3d tmp;
    tmp(0) = wp(0, i);
    tmp(1) = wp(1, i);
    tmp(2) = wp(2, i);
    res.push_back(tmp);
  }
  res.push_back(X_0_t.translation());
  return res;
}

void TrajectoryTask::generateBS()
{
  bspline.reset(new mc_trajectory::BSplineTrajectory(controlPoints(), duration));
}

Eigen::VectorXd TrajectoryTask::eval() const
{
  const auto& robot = robots.robot(rIndex);
  sva::PTransformd X_0_s = robot.surface(surfaceName).X_0_s(robot);
  return sva::transformError(X_0_s, X_0_t).vector();
}

Eigen::VectorXd TrajectoryTask::evalTracking() const
{
  return transTask->eval();
}

Eigen::VectorXd TrajectoryTask::speed() const
{
  return transTask->speed();
}

bool TrajectoryTask::timeElapsed()
{
  return t >= duration;
}

void TrajectoryTask::update()
{
  auto res = bspline->splev({t}, 2);
  Eigen::Vector3d & pos = res[0][0];
  Eigen::Vector3d & vel = res[0][1];
  Eigen::Vector3d & acc = res[0][2];

  // Change orientation waypoint
  if(t == 0 || t > oriStartTime+oriDuration)
  {
    const auto oriTargetTime = oriWpTime_[oriTargetWpIndex];
    oriDuration = oriTargetTime - t;
    oriStartTime = t;

    // Start from current surface pose
    const mc_rbdyn::Robot& robot = robots.robot(rIndex);
    const auto& surface = robot.surface(surfaceName);
    X_0_oriStart = surface.X_0_s(robot);

    const Eigen::Matrix3d& R_0_oriTarget = oriWp_[oriTargetWpIndex];
    // Get position along bspline at orientation waypoint time
    const auto t_0_oriTarget = bspline->splev({oriTargetTime}, 0)[0][0];
    X_0_oriTarget = sva::PTransformd(R_0_oriTarget, t_0_oriTarget);

    if(t > 0) ++oriTargetWpIndex;
  }
  // Interpolate rotation between waypoints
  sva::PTransformd interp = sva::interpolate(X_0_oriStart, X_0_oriTarget, (t-oriStartTime) / oriDuration);
  sva::PTransformd target(interp.rotation(), pos);

  // Set the trajectory tracking task targets from the trajectory.
  Eigen::VectorXd refVel(6);
  Eigen::VectorXd refAcc(6);
  for(unsigned int i = 0; i < 3; ++i)
  {
    refVel(i) = 0;
    refAcc(i) = 0;
    refVel(i+3) = vel(i);
    refAcc(i+3) = acc(i);
  }
  transTask->target(target);
  transTrajTask->refVel(refVel);
  transTrajTask->refAccel(refAcc);

  t = std::min(t + timeStep, duration);
}


void TrajectoryTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver)
  {
    solver.addTask(transTrajTask.get());
    inSolver = true;
  }
}

void TrajectoryTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver)
  {
    solver.removeTask(transTrajTask.get());
    inSolver = false;
  }
}

void TrajectoryTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                        const std::vector<std::string> & activeJoints)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  const auto stiff = stiffness();
  const auto damp = damping();
  const auto &dimW = dimWeight();
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(robots.mbs(), rIndex, transTask.get(), activeJoints));
  transTrajTask = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, 1.0);
  transTrajTask->dimWeight(dimW);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void TrajectoryTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                          const std::vector<std::string> & unactiveJoints)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  const auto stiff = stiffness();
  const auto damp = damping();
  const auto &dimW = dimWeight();
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(robots.mbs(), rIndex, transTask.get(), unactiveJoints));
  transTrajTask = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, 1.0);
  transTrajTask->dimWeight(dimW);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void TrajectoryTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  const auto stiff = stiffness();
  const auto damp = damping();
  const auto &dimW = dimWeight();
  selectorT = nullptr;
  transTrajTask = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, transTask.get(), stiff, damp, 1.0);
  transTrajTask->dimWeight(dimW);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void TrajectoryTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_surface_pose",
                     [this]()
                     {
                       const auto & robot = robots.robot(rIndex);
                       return robot.surface(surfaceName).X_0_s(robot);
                     });
  logger.addLogEntry(name_ + "_target_trajectory_pose",
                     [this]()
                     {
                       return this->transTask->target();
                     });
  logger.addLogEntry(name_ + "_target_pose",
                     [this]()
                     {
                       return this->X_0_t;
                     });
}

void TrajectoryTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_surface_pose");
  logger.removeLogEntry(name_ + "_target_trajectory_pose");
  logger.removeLogEntry(name_ + "_target_pose");
}

void TrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::Transform("pos_target",
                             [this]()
                             {
                             return this->target();
                             },
                             [this](const sva::PTransformd& pos)
                             {
                             target(pos);
                             }
                            ),
      mc_rtc::gui::Transform("traj_target",
                             [this]() { return this->transTask->target(); }),
      mc_rtc::gui::Transform("ori_target",
                             [this]() { return this->X_0_oriTarget; }),
      mc_rtc::gui::Transform("pos",
                             [this]()
                             {
                             return robots.robot(rIndex).surface(surfaceName).X_0_s(robots.robot(rIndex));
                             })
  );

  gui.addElement(
      {"Tasks", name_, "Gains"},
      mc_rtc::gui::NumberInput(
          "stiffness", [this]() { return this->stiffness(); },
          [this](const double& s) {
            this->setGains(s, this->damping());
          }),
      mc_rtc::gui::NumberInput(
          "damping", [this]() { return this->damping(); },
          [this](const double& d) {
            this->setGains(this->stiffness(), d);
          }),
      mc_rtc::gui::NumberInput(
          "stiffness & damping",
          [this]() { return this->stiffness(); },
          [this](const double& g) { this->stiffness(g); }),
      mc_rtc::gui::NumberInput("posWeight",
                               [this]() { return this->posWeight(); },
                               [this](const double& d) { this->posWeight(d); }),
      mc_rtc::gui::NumberInput(
          "oriWeight", [this]() { return this->oriWeight(); },
          [this](const double& g) { this->oriWeight(g); })
  );

  // Visual controls for the control points and
  for (unsigned int i = 0; i < wp.cols(); ++i)
  {
    gui.addElement(
        {"Tasks", name_},
        mc_rtc::gui::Point3D(
            "control_point_" + std::to_string(i),
            [this, i]() { return Eigen::Vector3d(this->wp.col(i)); },
            [this, i](const Eigen::Vector3d& pos) {
              wp.col(i) = pos;
              generateBS();
            })
    );
  }
}

}

namespace
{
static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "trajectoryTask",
    [](mc_solver::QPSolver& solver, const mc_rtc::Configuration& config)
    {

      sva::PTransformd X_0_t;
      unsigned int nrWP = 0;
      Eigen::MatrixXd waypoints;
      std::vector<double> oriWpTime;
      std::vector<Eigen::Matrix3d> oriWp;
      const auto robotIndex = config("robotIndex");

      if(config.has("nrWP"))
      {
        nrWP = config("nrWP");
      }

      if (config.has("targetSurface"))
      { // Target defined from a target surface, with an offset defined
        // in the surface coordinates
        const auto& c = config("targetSurface");
        const auto& surfaceName = c("surface");
        const auto& robot = solver.robot(c("robotIndex"));

        auto targetRelative = [&robot, &surfaceName](const Eigen::Vector3d offset_trans, const Eigen::Vector3d& offset_rpy, const sva::PTransformd X_0_t)
        {
          using namespace Eigen;
          Matrix3d m;
          m = AngleAxisd(offset_rpy.x() * M_PI / 180., Vector3d::UnitX()) *
              AngleAxisd(offset_rpy.y() * M_PI / 180., Vector3d::UnitY()) *
              AngleAxisd(offset_rpy.z() * M_PI / 180., Vector3d::UnitZ());
          sva::PTransformd offset = sva::PTransformd(m.inverse(), offset_trans);
          return offset * X_0_t;
        };

        sva::PTransformd targetSurface = robot.surface(surfaceName).X_0_s(robot);
        if (c.has("offset"))
        {
          const auto& o = c("offset");
          Eigen::Vector3d trans = o("translation");
          Eigen::Vector3d rpy = o("rotation");
          X_0_t = targetRelative(trans, rpy, targetSurface);
        }
        else
        {
          X_0_t = targetSurface;
        }

        if (c.has("waypoints"))
        {
          const auto& cw = c("waypoints");
          // Control points offsets defined wrt to the target surface frame
          const auto& controlPoints = cw("controlPoints");

          nrWP = 0; // No automatic waypoint, provide control points
          waypoints.resize(3, controlPoints.size());
          for (unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            const Eigen::Vector3d wp = controlPoints[i];
            sva::PTransformd X_offset(wp);
            waypoints.block(0, i, 3, 1) = (X_offset * targetSurface).translation();
          }


          if(cw.has("oriWaypoints"))
          {
            LOG_INFO("has orientation waypoints!");
            const auto& oc = cw("oriWaypoints");
            oriWpTime = oc("time");
            std::vector<Eigen::Vector3d> ori_rpy = oc("rotation");
            for(const auto & rpy : ori_rpy)
            {
              const auto &ori = targetRelative({0,0,0}, rpy, targetSurface);
              oriWp.push_back(ori.rotation());
            }
          }
        }
      }
      else if(config.has("target"))
      { // Absolute target pose
        X_0_t = config("target");

        if(config.has("waypoints"))
        {
          const auto& c = config("waypoints");
          // Control points defined in world coordinates
          const auto& controlPoints = c("controlPoints");
          nrWP = 0; // No automatic waypoint, provide control points
          waypoints.resize(3, controlPoints.size());
          waypoints.resize(3, controlPoints.size());
          for (unsigned int i = 0; i < controlPoints.size(); ++i)
          {
            const Eigen::Vector3d wp = controlPoints[i];
            waypoints.block<3, 1>(0, i) = wp;
          }

          if(c.has("oriWaypoints"))
          {
            const auto& oc = c("oriWaypoints");
            oriWpTime = oc("time");
            std::vector<Eigen::Vector3d> ori_rpy = oc("rotation");
            for(const auto & rpy : ori_rpy)
            {
              using namespace Eigen;
              Matrix3d m;
              m = AngleAxisd(rpy.x() * M_PI / 180., Vector3d::UnitX()) *
                  AngleAxisd(rpy.y() * M_PI / 180., Vector3d::UnitY()) *
                  AngleAxisd(rpy.z() * M_PI / 180., Vector3d::UnitZ());
              oriWp.push_back(m.inverse());
            }
          }
        }

      }


      auto t = std::make_shared<mc_tasks::TrajectoryTask>(
          solver.robots(),
          robotIndex,
          config("surface"),
          X_0_t,
          config("duration"),
          config("timeStep"),
          config("stiffness"),
          config("posWeight"),
          config("oriWeight"),
          waypoints,
          oriWpTime,
          oriWp,
          nrWP
          );
      t->load(solver, config);
      return t;
    });
}
