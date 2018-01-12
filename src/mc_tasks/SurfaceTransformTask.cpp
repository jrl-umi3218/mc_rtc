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

  type_ = "surfaceTransform";
  name_ = "surface_transform_" + robot.name() + "_" + surfaceName;
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

void SurfaceTransformTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_surface_pose",
                     [this]()
                     {
                       const auto & robot = robots.robot();
                       return robot.surface(surfaceName).X_0_s(robot);
                     });
  logger.addLogEntry(name_ + "_target_pose",
                     [this]()
                     {
                       return target();
                     });
}

void SurfaceTransformTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_surface_pose");
  logger.removeLogEntry(name_ + "_target_pose");
}

void SurfaceTransformTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);
  gui.addElement(
    mc_rtc::gui::Element<sva::PTransformd>{
      {"Tasks", name_, "pos_target"},
      [this](){ return this->target(); },
      [this](const sva::PTransformd & pos) { this->target(pos); }
    },
    mc_rtc::gui::Transform{}
  );
  gui.addElement(
    mc_rtc::gui::Element<sva::PTransformd>{
      {"Tasks", name_, "pos"},
      std::function<sva::PTransformd()>{
        [this]()
        {
          return robots.robot(rIndex).surface(surfaceName).X_0_s(robots.robot(rIndex));
        }
      }
    },
    mc_rtc::gui::Transform{}
  );
}

}

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("surfaceTransform",
  [](mc_solver::QPSolver & solver,
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
);

}
