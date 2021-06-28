/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/rpy_utils.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/MomentumTask.h>

#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

MomentumTask::MomentumTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::MomentumTask>(robots, robotIndex, stiffness, weight)
{
  auto & robot = robots.robot(robotIndex);
  auto momentum = rbd::computeCentroidalMomentum(robot.mb(), robot.mbc(), robot.com());
  finalize(robots.mbs(), static_cast<int>(rIndex), momentum);

  type_ = "momentum";
  name_ = "momentum_" + robots.robot(robotIndex).name();
}

void MomentumTask::reset()
{
  TrajectoryTaskGeneric::reset();
  const auto & robot = robots.robot(rIndex);
  errorT->momentum(rbd::computeCentroidalMomentum(robot.mb(), robot.mbc(), robot.com()));
}

/*! \brief Load parameters from a Configuration object */
void MomentumTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("momentum"))
  {
    this->momentum(config("momentum"));
  }
  TrajectoryBase::load(solver, config);
}

sva::ForceVecd MomentumTask::momentum() const
{
  return errorT->momentum();
}

void MomentumTask::momentum(const sva::ForceVecd & m)
{
  errorT->momentum(m);
}

void MomentumTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target_momentum", this, [this]() { return momentum(); });
  // FIXME Not correct with dimWeight
  logger.addLogEntry(name_ + "_momentum", this,
                     [this]() { return sva::ForceVecd(momentum().vector() - errorT->eval()); });
}

void MomentumTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<tasks::qp::MomentumTask>::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::ArrayInput(
                     "target", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() { return this->momentum(); },
                     [this](const sva::ForceVecd & m) { this->momentum(m); }),
                 mc_rtc::gui::ArrayLabel("momentum", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> Eigen::Vector6d {
                   return this->momentum().vector() - this->errorT->eval();
                 }));
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "momentum",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto robotIndex = robotIndexFromConfig(config, solver.robots(), "MomentumTask");
      auto t = std::make_shared<mc_tasks::MomentumTask>(solver.robots(), robotIndex);
      t->load(solver, config);
      return t;
    });
}
