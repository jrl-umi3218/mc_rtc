#include <mc_tasks/LookAtTask.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <tf2_eigen/tf2_eigen.h>

namespace mc_tasks
{
LookAtTask::LookAtTask(const std::string& bodyName,
                       const Eigen::Vector3d& bodyVector,
                       const Eigen::Vector3d& targetPos,
                       const mc_rbdyn::Robots& robots, unsigned int robotIndex,
                       double stiffness, double weight)
    : VectorOrientationTask(bodyName, bodyVector, bodyVector, robots,
                            robotIndex, stiffness, weight)
{
  const mc_rbdyn::Robot& robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, bodyVector,
           targetPos);
  type_ = "lookAt";
  name_ = "look_at_" + robot.name() + "_" + bodyName;
}

void LookAtTask::reset()
{
  VectorOrientationTask::reset();
  target_pos_ = Eigen::Vector3d::Zero();
}

void LookAtTask::target(const Eigen::Vector3d& pos)
{
  target_pos_ = pos;
  const sva::PTransformd& X_0_b = robots.robot(rIndex).mbc().bodyPosW[bIndex];
  auto target_ori = (pos - X_0_b.translation()).normalized();
  VectorOrientationTask::targetVector(target_ori);
}

Eigen::Vector3d LookAtTask::target() const { return target_pos_; }
void LookAtTask::addToLogger(mc_rtc::Logger& logger)
{
  logger.addLogEntry(name_ + "_target", [this]() -> const Eigen::Vector3d {
    return VectorOrientationTask::targetVector();
  });
  logger.addLogEntry(name_ + "_current", [this]() -> Eigen::Vector3d {
    return VectorOrientationTask::actual();
  });
  logger.addLogEntry(name_ + "_error",
                     [this]() -> Eigen::Vector3d { return eval(); });
}

LookAtSurfaceTask::LookAtSurfaceTask(const mc_rbdyn::Robots& robots,
                                     unsigned int robotIndex,
                                     const std::string& bodyName,
                                     const Eigen::Vector3d& bodyVector,
                                     unsigned int surfaceRobotIndex,
                                     const std::string& surfaceName,
                                     double stiffness, double weight)
    : LookAtTask(bodyName, bodyVector, bodyVector, robots, robotIndex,
                 stiffness, weight),
      sRobotIndex(surfaceRobotIndex),
      sName(surfaceName)
{
  const mc_rbdyn::Robot& robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, bodyVector,
           bodyVector);
  type_ = "lookAtSurface";
  name_ =
      "look_at_surface_" + robot.name() + "_" + bodyName + "_" + surfaceName;
}

void LookAtSurfaceTask::update()
{
  auto& robot = robots.robot(sRobotIndex);
  LookAtTask::target(robot.surface(sName).X_0_s(robot).translation());
}

} /* mc_tasks */

namespace
{
static bool registered_lookat = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAt",
    [](mc_solver::QPSolver& solver, const mc_rtc::Configuration& config) {
      auto t = std::make_shared<mc_tasks::LookAtTask>(
          config("body"), config("bodyVector"), config("targetVector"),
          solver.robots(), config("robotIndex"));
      if (config.has("weight"))
      {
        t->weight(config("weight"));
      }
      if (config.has("stiffness"))
      {
        t->stiffness(config("stiffness"));
      }
      t->load(solver, config);
      return t;
    });

static bool registered_lookat_surface =
    mc_tasks::MetaTaskLoader::register_load_function(
        "lookAtSurface",
        [](mc_solver::QPSolver& solver, const mc_rtc::Configuration& config) {
          auto t = std::make_shared<mc_tasks::LookAtSurfaceTask>(
              solver.robots(), config("robotIndex"), config("bodyName"),
              config("bodyVector"), config("surfaceRobotIndex"),
              config("surfaceName"));
          if (config.has("weight"))
          {
            t->weight(config("weight"));
          }
          if (config.has("stiffness"))
          {
            t->stiffness(config("stiffness"));
          }
          t->load(solver, config);
          return t;
        });
}
