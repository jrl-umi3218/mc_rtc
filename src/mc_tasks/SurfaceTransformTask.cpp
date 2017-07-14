#include <mc_tasks/SurfaceTransformTask.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rbdyn/configuration_io.h>

namespace mc_tasks
{

SurfaceTransformTask::SurfaceTransformTask(const std::string & surfaceName, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>(robots, robotIndex, stiffness, weight),
  surfaceName(surfaceName)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  std::string bodyName = robot.surface(surfaceName).bodyName();
  sva::PTransformd curPos = robot.surface(surfaceName).X_0_s(robot);
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, curPos, robot.surface(surfaceName).X_b_s());
}

void SurfaceTransformTask::reset()
{
  const auto & robot = robots.robot(rIndex);
  sva::PTransformd curPos = robot.surface(surfaceName).X_0_s(robot);
  errorT->target(curPos);
}

sva::PTransformd SurfaceTransformTask::target()
{
  return errorT->target();
}

void SurfaceTransformTask::target(const sva::PTransformd & pose)
{
  errorT->target(pose);
}

}

namespace
{

mc_tasks::MetaTaskPtr load_surface_transform_task(mc_solver::QPSolver & solver,
                                                  const mc_rtc::Configuration & config)
{
  auto t = std::make_shared<mc_tasks::SurfaceTransformTask>(config("surface"), solver.robots(), config("robotIndex"));
  if(config.has("target"))
  {
    t->target(config("target"));
  }
  t->load(solver, config);
  return t;
}

struct SurfaceTransformLoader
{
  static bool registered;
};

bool SurfaceTransformLoader::registered = mc_tasks::MetaTaskLoader::register_load_function("surfaceTransform", &load_surface_transform_task);

}
