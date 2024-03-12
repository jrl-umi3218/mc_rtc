/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/MomentumTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_tvm/MomentumFunction.h>

#include <mc_rbdyn/rpy_utils.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

static inline mc_rtc::void_ptr_caster<tasks::qp::MomentumTask> tasks_error{};
static inline mc_rtc::void_ptr_caster<mc_tvm::MomentumFunction> tvm_error{};

MomentumTask::MomentumTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: TrajectoryTaskGeneric(robots, robotIndex, stiffness, weight)
{
  auto & robot = robots.robot(robotIndex);
  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto momentum = rbd::computeCentroidalMomentum(robot.mb(), robot.mbc(), robot.com());
      finalize<Backend::Tasks, tasks::qp::MomentumTask>(robots.mbs(), static_cast<int>(rIndex), momentum);
      break;
    }
    case Backend::TVM:
      finalize<Backend::TVM, mc_tvm::MomentumFunction>(robot);
      break;
    default:
      mc_rtc::log::error_and_throw("[MomentumTask] Not implemented for solver backend: {}", backend_);
  }
  type_ = "momentum";
  name_ = "momentum_" + robots.robot(robotIndex).name();
}

void MomentumTask::reset()
{
  TrajectoryTaskGeneric::reset();
  const auto & robot = robots.robot(rIndex);
  momentum(rbd::computeCentroidalMomentum(robot.mb(), robot.mbc(), robot.com()));
}

/*! \brief Load parameters from a Configuration object */
void MomentumTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("momentum")) { this->momentum(config("momentum")); }
  TrajectoryBase::load(solver, config);
}

sva::ForceVecd MomentumTask::momentum() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(errorT)->momentum();
    case Backend::TVM:
      return tvm_error(errorT)->momentum();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void MomentumTask::momentum(const sva::ForceVecd & m)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->momentum(m);
      break;
    case Backend::TVM:
      tvm_error(errorT)->momentum(m);
      break;
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void MomentumTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target_momentum", this, [this]() { return momentum(); });
  // FIXME Not correct with dimWeight
  switch(backend_)
  {
    case Backend::Tasks:
      logger.addLogEntry(name_ + "_momentum", this,
                         [this]() { return sva::ForceVecd(momentum().vector() - tasks_error(errorT)->eval()); });
      break;
    case Backend::TVM:
      logger.addLogEntry(name_ + "_momentum", this,
                         [this]() -> const sva::ForceVecd & { return tvm_error(errorT)->algo().momentum(); });
      break;
    default:
      break;
  }
}

void MomentumTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::ArrayInput(
                     "target", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() { return this->momentum(); },
                     [this](const sva::ForceVecd & m) { this->momentum(m); }));
  switch(backend_)
  {
    case Backend::Tasks:
    {
      gui.addElement({"Tasks", name_}, mc_rtc::gui::ArrayLabel(
                                           "momentum", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> Eigen::Vector6d
                                           { return this->momentum().vector() - tasks_error(this->errorT)->eval(); }));
      break;
    }
    case Backend::TVM:
    {
      gui.addElement({"Tasks", name_}, mc_rtc::gui::ArrayLabel("momentum", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                                               [this]() -> const sva::ForceVecd &
                                                               { return tvm_error(errorT)->algo().momentum(); }));
      break;
    }
    default:
      break;
  }
}

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "momentum",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto robotIndex = robotIndexFromConfig(config, solver.robots(), "MomentumTask");
      auto t = std::make_shared<mc_tasks::MomentumTask>(solver.robots(), robotIndex);
      t->load(solver, config);
      return t;
    });
} // namespace
