#include <mc_tasks/LookAtTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{
LookAtTask::LookAtTask(const std::string & bodyName,
                       const Eigen::Vector3d & bodyVector,
                       const Eigen::Vector3d & targetPos,
                       const mc_rbdyn::Robots & robots,
                       unsigned int robotIndex,
                       double stiffness,
                       double weight)
: VectorOrientationTask(bodyName, bodyVector, bodyVector, robots, robotIndex, stiffness, weight)
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);

  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, bodyVector, Eigen::Vector3d{1.,0.,0.});
  LookAtTask::target(targetPos);
  type_ = "lookAt";
  name_ = "look_at_" + robot.name() + "_" + bodyName;
}

void LookAtTask::reset()
{
  VectorOrientationTask::reset();
  target_pos_ = Eigen::Vector3d::Zero();
}

void LookAtTask::target(const Eigen::Vector3d & pos)
{
  target_pos_ = pos;
  const sva::PTransformd & X_0_b = robots.robot(rIndex).mbc().bodyPosW[bIndex];
  auto target_ori = (pos - X_0_b.translation()).normalized();
  VectorOrientationTask::targetVector(target_ori);
}

Eigen::Vector3d LookAtTask::target() const
{
  return target_pos_;
}
void LookAtTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target",
                     [this]() -> const Eigen::Vector3d { return VectorOrientationTask::targetVector(); });
  logger.addLogEntry(name_ + "_current", [this]() -> Eigen::Vector3d { return VectorOrientationTask::actual(); });
  logger.addLogEntry(name_ + "_error", [this]() -> Eigen::Vector3d { return eval(); });
}

void LookAtTask::removeFromLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_target");
  logger.removeLogEntry(name_ + "_current");
  logger.removeLogEntry(name_ + "_error");
}


void LookAtTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  VectorOrientationTask::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Point3D("target_point",
                                      [this]()
                                      {
                                        return this->target();
                                      },
                                      [this](const Eigen::Vector3d& pos)
                                      {
                                        this->target(pos);
                                      }),
                 mc_rtc::gui::Arrow("robot_vector",
                                    mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(1., 0., 0.)),
                                    [this]() -> Eigen::Vector3d
                                    {
                                      return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation();
                                    },
                                    [this]() -> Eigen::Vector3d
                                    {
                                      return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation() + .25 * targetVector();
                                    }),
                 mc_rtc::gui::Arrow("target_vector",
                                    mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 1., 0.)),
                                    [this]() -> Eigen::Vector3d
                                    {
                                      return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation();
                                    },
                                    [this]() -> Eigen::Vector3d
                                    {
                                      return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation() + .25 * bodyVector();
                                    })
                 );
}

} // namespace mc_tasks

namespace
{
static bool registered_lookat = mc_tasks::MetaTaskLoader::register_load_function(
    "lookAt",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::LookAtTask>(config("body"), config("bodyVector"), config("targetPos"),
                                                      solver.robots(), config("robotIndex"));
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
      t->load(solver, config);
      return t;
    });
}
