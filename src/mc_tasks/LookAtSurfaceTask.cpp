/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/LookAtSurfaceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

LookAtSurfaceTask::LookAtSurfaceTask(const mc_rbdyn::Robots & robots,
                                     unsigned int robotIndex,
                                     const std::string & bodyName,
                                     const Eigen::Vector3d & bodyVector,
                                     unsigned int surfaceRobotIndex,
                                     const std::string & surfaceName,
                                     double stiffness,
                                     double weight)
: LookAtTask(bodyName, bodyVector, robots, robotIndex, stiffness, weight), sRobotIndex(surfaceRobotIndex),
  sName(surfaceName), offset_(sva::PTransformd::Identity())
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, bodyVector, bodyVector);
  type_ = "lookAtSurface";
  name_ = "look_at_surface_" + robot.name() + "_" + bodyName + "_" + surfaceName;
}

void LookAtSurfaceTask::update(mc_solver::QPSolver &)
{
  auto & robot = robots.robot(sRobotIndex);
  LookAtTask::target((offset_ * robot.surfacePose(sName)).translation());
}

const sva::PTransformd & LookAtSurfaceTask::offset() const
{
  return offset_;
}

void LookAtSurfaceTask::offset(const sva::PTransformd & off)
{
  offset_ = off;
}

} // namespace mc_tasks

namespace
{

static auto registered_lookat_surface = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAtSurface",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::LookAtSurfaceTask>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "lookAtSurface"), config("body"),
          config("bodyVector"), config("surfaceRobotIndex"), config("surface"));
      if(config.has("weight"))
      {
        t->weight(config("weight"));
      }
      if(config.has("stiffness"))
      {
        auto s = config("stiffness");
        if(s.size())
        {
          Eigen::VectorXd st = s;
          t->stiffness(st);
        }
        else
        {
          t->stiffness(static_cast<double>(s));
        }
      }
      if(config.has("offset"))
      {
        t->offset(config("offset"));
      }
      t->load(solver, config);
      return t;
    });
}
