#include <mc_tasks/PositionBasedVisServoTask.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rbdyn/configuration_io.h>

namespace mc_tasks{

PositionBasedVisServoTask::PositionBasedVisServoTask(const std::string& bodyName,
    const sva::PTransformd& X_t_s, const sva::PTransformd& X_b_s,
    const mc_rbdyn::Robots & robots, unsigned int robotIndex,
    double stiffness, double weight)
  : TrajectoryTaskGeneric<tasks::qp::PositionBasedVisServoTask>(robots, robotIndex, stiffness, weight)
{
    finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, X_t_s, X_b_s);
    name_ = "pbvs_" + robots.robot(robotIndex).name() + "_" + bodyName;
}

PositionBasedVisServoTask::PositionBasedVisServoTask(const std::string& surfaceName,
    const sva::PTransformd& X_t_s,
    const mc_rbdyn::Robots & robots, unsigned int robotIndex,
    double stiffness, double weight)
: PositionBasedVisServoTask(robots.robot(robotIndex).surface(surfaceName).bodyName(),
                            X_t_s,
                            robots.robot(robotIndex).surface(surfaceName).X_b_s(),
                            robots, robotIndex, stiffness, weight)
{
}

void PositionBasedVisServoTask::reset()
{
    errorT->error(sva::PTransformd::Identity());
}

void PositionBasedVisServoTask::error(const sva::PTransformd& X_t_s)
{
  errorT->error(X_t_s);
}

} // namespace mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("pbvs",
  [](mc_solver::QPSolver & solver,
     const mc_rtc::Configuration & config)
  {
    auto t = std::make_shared<mc_tasks::PositionBasedVisServoTask>(config("surface"), sva::PTransformd::Identity(), solver.robots(), config("robotIndex"));
    t->load(solver, config);
    return t;
  }
);

}
