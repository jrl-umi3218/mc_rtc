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

mc_tasks::MetaTaskPtr load_pbvs_task(mc_solver::QPSolver & solver,
                                    const mc_rtc::Configuration & config)
{
  auto t = std::make_shared<mc_tasks::PositionBasedVisServoTask>(config("body"), sva::PTransformd::Identity(), config("X_b_s"), solver.robots(), config("robotIndex"));
  t->load(solver, config);
  return t;
}

struct PositionBasedVisServoLoader
{
  static bool registered;
};

bool PositionBasedVisServoLoader::registered = mc_tasks::MetaTaskLoader::register_load_function("pbvs", &load_pbvs_task);

}
