#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/VectorOrientationTask.h>

namespace mc_tasks
{

VectorOrientationTask::VectorOrientationTask(const std::string & bodyName,
                                             const Eigen::Vector3d & bodyVector,
                                             const Eigen::Vector3d & targetVector,
                                             const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             double stiffness,
                                             double weight)
: TrajectoryTaskGeneric<tasks::qp::VectorOrientationTask>(robots, robotIndex, stiffness, weight), bodyName(bodyName),
  bIndex(0)
{
  init();
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, bodyVector, targetVector);
}

VectorOrientationTask::VectorOrientationTask(const std::string & bodyName,
                                             const Eigen::Vector3d & bodyVector,
                                             const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             double stiffness,
                                             double weight)
: TrajectoryTaskGeneric<tasks::qp::VectorOrientationTask>(robots, robotIndex, stiffness, weight), bodyName(bodyName),
  bIndex(0)
{
  init();

  const auto & robot = robots.robot();
  Eigen::Matrix3d E_0_b = robot.mbc().bodyPosW[bIndex].rotation().transpose();
  Eigen::Vector3d actualVector = E_0_b * bodyVector;

  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, bodyVector, actualVector);
}

void VectorOrientationTask::init()
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  bIndex = robot.bodyIndexByName(bodyName);
  type_ = "vectorOrientation";
  name_ = "vector_orientation_" + robot.name() + "_" + bodyName;
}

void VectorOrientationTask::reset()
{
  TrajectoryTaskGeneric::reset();
  errorT->target(errorT->actual());
}

void VectorOrientationTask::bodyVector(const Eigen::Vector3d & vector)
{
  errorT->bodyVector(vector);
}

Eigen::Vector3d VectorOrientationTask::bodyVector() const
{
  return errorT->bodyVector();
}

void VectorOrientationTask::targetVector(const Eigen::Vector3d & ori)
{
  errorT->target(ori);
}

Eigen::Vector3d VectorOrientationTask::targetVector() const
{
  return errorT->target();
}

Eigen::Vector3d VectorOrientationTask::actual() const
{
  return errorT->actual();
}

void VectorOrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Arrow(
                     "robot_vector", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 0., 1.)),
                     [this]() -> Eigen::Vector3d { return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation(); },
                     [this]() -> Eigen::Vector3d {
                       return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation() + .25 * actual();
                     }),
                 mc_rtc::gui::Arrow(
                     "target_vector", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(1., 0., 0.)),
                     [this]() -> Eigen::Vector3d { return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation(); },
                     [this]() -> Eigen::Vector3d {
                       return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation() + .25 * targetVector();
                     }));
}
} // namespace mc_tasks

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function(
    "vectorOrientation",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::VectorOrientationTask>(config("body"), config("bodyVector"), solver.robots(),
                                                                 config("robotIndex"));
      t->load(solver, config);
      if(config.has("targetVector"))
      {
        t->targetVector(config("targetVector"));
      }
      return t;
    });
}
