#include <mc_tasks/LookAtSurfaceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

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

static bool registered_lookat_surface =
    mc_tasks::MetaTaskLoader::register_load_function(
        "lookAtSurface",
        [](mc_solver::QPSolver& solver, const mc_rtc::Configuration& config) {
          auto t = std::make_shared<mc_tasks::LookAtSurfaceTask>(
              solver.robots(), config("robotIndex"), config("body"),
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

