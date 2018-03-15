#include <mc_tasks/VectorOrientationTask.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

VectorOrientationTask::VectorOrientationTask(const std::string & bodyName, const Eigen::Vector3d& bodyVector,
		const Eigen::Vector3d& targetVector, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::VectorOrientationTask>(robots, robotIndex, stiffness, weight),
  bodyName(bodyName), bIndex(0)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, bodyVector, targetVector);
  name_ = "vector_orientation_" + robot.name() + "_" + bodyName;
}

void VectorOrientationTask::reset()
{
  errorT->target(errorT->actual());
}

void VectorOrientationTask::bodyVector(const Eigen::Vector3d & vector)
{
  errorT->bodyVector(vector);
}

Eigen::Vector3d VectorOrientationTask::bodyVector()
{
  return errorT->bodyVector();
}

void VectorOrientationTask::targetVector(const Eigen::Vector3d & vector)
{
  errorT->target(vector);
}

const Eigen::Vector3d & VectorOrientationTask::targetVector() const
{
  return errorT->target();
}

}

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("vectorOrientation",
  [](mc_solver::QPSolver & solver,
     const mc_rtc::Configuration & config)
  {
    auto t = std::make_shared<mc_tasks::VectorOrientationTask>(config("body"), config("bodyVector"), config("targetVector"), solver.robots(), config("robotIndex"));
    t->load(solver, config);
    return t;
  }
);

}
