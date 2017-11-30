#include <Eigen/Geometry>

#include <mc_rbdyn/configuration_io.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/PositionBasedVisServoTask.h>

namespace mc_tasks{

PositionBasedVisServoTask::PositionBasedVisServoTask(const std::string& bodyName,
    const sva::PTransformd& X_t_s, const sva::PTransformd& X_b_s,
    const mc_rbdyn::Robots & robots, unsigned int robotIndex,
    double stiffness, double weight)
  : TrajectoryTaskGeneric<tasks::qp::PositionBasedVisServoTask>(robots, robotIndex, stiffness, weight)
{
    finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, X_t_s, X_b_s);
    type_ = "pbvs";
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
  X_t_s_ = sva::PTransformd::Identity();
  errorT->error(X_t_s_);
}

void PositionBasedVisServoTask::error(const sva::PTransformd& X_t_s)
{
  X_t_s_ = X_t_s;
  errorT->error(X_t_s_);
}

void PositionBasedVisServoTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_error",
                     [this]() -> const sva::PTransformd &
                     {
                     return X_t_s_;
                     });
  logger.addLogEntry(name_ + "_eval",
                     [this]() -> sva::PTransformd
                     {
                     Eigen::Vector6d eval = errorT->eval();
                     Eigen::Vector3d angleAxis = eval.head(3);
                     Eigen::Vector3d axis = angleAxis / angleAxis.norm();
                     double angle = angleAxis.dot(axis);
                     Eigen::Quaterniond quat(Eigen::AngleAxisd(angle, axis));
                     return sva::PTransformd(quat, eval.tail(3));
                     });
}

void PositionBasedVisServoTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_error");
  logger.removeLogEntry(name_ + "_eval");
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
