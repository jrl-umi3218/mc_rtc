/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Surface.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TrajectoryTask.h>
#include <mc_trajectory/spline_utils.h>

namespace mc_tasks
{
TrajectoryTask::TrajectoryTask(const mc_rbdyn::Robots & robots,
                               unsigned int robotIndex,
                               const std::string & surfaceName,
                               double duration,
                               double stiffness,
                               double posW,
                               double oriW)
: robots(robots)
{
  this->rIndex = robotIndex;
  this->surfaceName = surfaceName;
  this->X_0_t = X_0_t;
  this->duration = duration;
  stiffness_ = stiffness;
  damping_ = 2 * sqrt(stiffness_);

  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  const auto & surface = robot.surface(surfaceName);
  type_ = "trajectory";
  name_ = "trajectory_" + robot.name() + "_" + surface.name();
  X_0_start = surface.X_0_s(robot);

  transTask.reset(new tasks::qp::TransformTask(robots.mbs(), static_cast<int>(robotIndex), surface.bodyName(),
                                               X_0_start, surface.X_b_s()));
  transTrajTask.reset(new tasks::qp::TrajectoryTask(robots.mbs(), static_cast<int>(robotIndex), transTask.get(),
                                                    stiffness, 2 * sqrt(stiffness), 1.0));
  posWeight(posW);
  oriWeight(oriW);
}

void TrajectoryTask::stiffness(double s)
{
  stiffness_ = s;
  setGains(s, 2 * std::sqrt(s));
}

void TrajectoryTask::damping(double d)
{
  damping_ = d;
  setGains(stiffness(), d);
}

void TrajectoryTask::setGains(double s, double d)
{
  transTrajTask->setGains(s, d);
}

double TrajectoryTask::stiffness() const
{
  return stiffness_;
}

double TrajectoryTask::damping() const
{
  return damping_;
}

void TrajectoryTask::posWeight(const double posWeight)
{
  auto dimWeight = transTrajTask->dimWeight();
  dimWeight.tail(3).setConstant(posWeight);
  transTrajTask->dimWeight(dimWeight);
}

double TrajectoryTask::posWeight() const
{
  return transTrajTask->dimWeight()(3);
}

void TrajectoryTask::oriWeight(const double oriWeight)
{
  auto dimWeight = transTrajTask->dimWeight();
  dimWeight.head(3).setConstant(oriWeight);
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

void TrajectoryTask::target(const sva::PTransformd & target)
{
  X_0_t = target;
}

const sva::PTransformd & TrajectoryTask::target() const
{
  return X_0_t;
}

// void TrajectoryTask::generateBS()
// {
//   // bspline.reset(new mc_trajectory::BSplineTrajectory(controlPoints(), duration));
//   // bspline.reset(new mc_trajectory::BSplineConstrainedTrajectory(controlPoints(), duration));
//   mc_trajectory::ExactCubicTrajectory::T_Waypoint waypoints;
//   int i = 0;
//   const auto & cps = controlPoints();
//   for(const auto & cp : cps)
//   {
//     double step = duration / (cps.size() - 1);
//     double t = i * step;
//     waypoints.push_back(std::make_pair(t, cp));
//     ++i;
//   }
//   bspline.reset(new mc_trajectory::ExactCubicTrajectory(waypoints, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
//                                                         Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()));
// }

Eigen::VectorXd TrajectoryTask::eval() const
{
  const auto & robot = robots.robot(rIndex);
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

void TrajectoryTask::displaySamples(unsigned s)
{
  samples_ = s;
}

unsigned TrajectoryTask::displaySamples() const
{
  return samples_;
}

void TrajectoryTask::refVel(const Eigen::VectorXd & vel)
{
  transTrajTask->refVel(vel);
}

const Eigen::VectorXd & TrajectoryTask::refVel() const
{
  return transTrajTask->refVel();
}

void TrajectoryTask::refAcc(const Eigen::VectorXd & acc)
{
  transTrajTask->refAccel(acc);
}

const Eigen::VectorXd & TrajectoryTask::refAcc() const
{
  return transTrajTask->refAccel();
}
void TrajectoryTask::refTarget(const sva::PTransformd & target)
{
  transTask->target(target);
}
const sva::PTransformd & TrajectoryTask::refTarget() const
{
  return transTask->target();
}

void TrajectoryTask::update()
{
  t = std::min(t + timeStep, duration);
}

void TrajectoryTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver)
  {
    timeStep = solver.dt();
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
                                        const std::vector<std::string> & activeJoints,
                                        const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  const auto stiff = stiffness();
  const auto damp = damping();
  const auto & dimW = dimWeight();
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(
      robots.mbs(), static_cast<int>(rIndex), transTask.get(), activeJoints, activeDofs));
  transTrajTask = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, 1.0);
  transTrajTask->dimWeight(dimW);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void TrajectoryTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                          const std::vector<std::string> & unactiveJoints,
                                          const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  const auto stiff = stiffness();
  const auto damp = damping();
  const auto & dimW = dimWeight();
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(
      robots.mbs(), static_cast<int>(rIndex), transTask.get(), unactiveJoints, unactiveDofs));
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
  const auto & dimW = dimWeight();
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
  logger.addLogEntry(name_ + "_surface_pose", [this]() {
    const auto & robot = robots.robot(rIndex);
    return robot.surface(surfaceName).X_0_s(robot);
  });
  logger.addLogEntry(name_ + "_target_trajectory_pose", [this]() { return this->transTask->target(); });
  logger.addLogEntry(name_ + "_target_pose", [this]() { return this->X_0_t; });
  logger.addLogEntry(name_ + "_target_vel", [this]() {
    const auto & refVel = this->transTrajTask->refVel();
    return sva::MotionVecd(refVel.head<3>(), refVel.tail<3>());
  });
  logger.addLogEntry(name_ + "_target_acc", [this]() {
    const auto & refAcc = this->transTrajTask->refAccel();
    return sva::MotionVecd(refAcc.head<3>(), refAcc.tail<3>());
  });
  logger.addLogEntry(name_ + "_speed", [this]() {
    const auto & speed = this->speed();
    return sva::MotionVecd(speed.head<3>(), speed.tail<3>());
  });
  logger.addLogEntry(name_ + "_normalAcc", [this]() {
    const auto & speed = this->transTask->normalAcc();
    return sva::MotionVecd(speed.head<3>(), speed.tail<3>());
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
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput("stiffness", [this]() { return this->stiffness(); },
                                          [this](const double & s) { this->setGains(s, this->damping()); }),
                 mc_rtc::gui::NumberInput("damping", [this]() { return this->damping(); },
                                          [this](const double & d) { this->setGains(this->stiffness(), d); }),
                 mc_rtc::gui::NumberInput("stiffness & damping", [this]() { return this->stiffness(); },
                                          [this](const double & g) { this->stiffness(g); }),
                 mc_rtc::gui::NumberInput("posWeight", [this]() { return this->posWeight(); },
                                          [this](const double & d) { this->posWeight(d); }),
                 mc_rtc::gui::NumberInput("oriWeight", [this]() { return this->oriWeight(); },
                                          [this](const double & g) { this->oriWeight(g); }));
}

} // namespace mc_tasks

// namespace
// {
// static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
//     "trajectory",
//     [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
//       sva::PTransformd X_0_t;
//       Eigen::MatrixXd waypoints;
//       std::vector<std::pair<double, Eigen::Matrix3d>> oriWp;
//       const auto robotIndex = config("robotIndex");

//       if(config.has("targetSurface"))
//       { // Target defined from a target surface, with an offset defined
//         // in the surface coordinates
//         const auto & c = config("targetSurface");
//         const auto & targetSurfaceName = c("surface");
//         const auto & robot = solver.robot(c("robotIndex"));

//         const sva::PTransformd & targetSurface = robot.surface(targetSurfaceName).X_0_s(robot);
//         const Eigen::Vector3d trans = c("offset_translation", Eigen::Vector3d::Zero().eval());
//         const Eigen::Matrix3d rot = c("offset_rotation", Eigen::Matrix3d::Identity().eval());
//         sva::PTransformd offset(rot, trans);
//         X_0_t = offset * targetSurface;

//         if(c.has("controlPoints"))
//         {
//           // Control points offsets defined wrt to the target surface frame
//           const auto & controlPoints = c("controlPoints");
//           waypoints.resize(3, controlPoints.size());
//           for(unsigned int i = 0; i < controlPoints.size(); ++i)
//           {
//             const Eigen::Vector3d wp = controlPoints[i];
//             sva::PTransformd X_offset(wp);
//             waypoints.col(i) = (X_offset * targetSurface).translation();
//           }
//         }

//         if(c.has("oriWaypoints"))
//         {
//           std::vector<std::pair<double, Eigen::Matrix3d>> oriWaypoints = c("oriWaypoints");
//           for(const auto & wp : oriWaypoints)
//           {
//             const sva::PTransformd offset{wp.second};
//             const sva::PTransformd ori = offset * targetSurface;
//             oriWp.push_back(std::make_pair(wp.first, ori.rotation()));
//           }
//         }
//       }
//       else
//       { // Absolute target pose
//         X_0_t = config("target");

//         if(config.has("controlPoints"))
//         {
//           // Control points defined in world coordinates
//           const auto & controlPoints = config("controlPoints");
//           waypoints.resize(3, controlPoints.size());
//           waypoints.resize(3, controlPoints.size());
//           for(unsigned int i = 0; i < controlPoints.size(); ++i)
//           {
//             const Eigen::Vector3d wp = controlPoints[i];
//             waypoints.col(i) = wp;
//           }
//         }

//         oriWp = config("oriWaypoints", std::vector<std::pair<double, Eigen::Matrix3d>>{});
//       }

//       std::shared_ptr<mc_tasks::TrajectoryTask> t = std::make_shared<mc_tasks::TrajectoryTask>(solver.robots(),
//       robotIndex, config("surface"), X_0_t,
//                                                        config("duration"), config("stiffness"), config("posWeight"),
//                                                        config("oriWeight"), waypoints, oriWp);
//       t->load(solver, config);
//       const auto displaySamples = config("displaySamples", t->displaySamples());
//       t->displaySamples(displaySamples);
//       return t;
//     });
// }
