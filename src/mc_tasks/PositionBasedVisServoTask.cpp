#include <mc_tasks/PositionBasedVisServoTask.h>

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
