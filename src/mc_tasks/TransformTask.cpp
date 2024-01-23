/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/TransformTask.h>

#include <mc_tvm/TransformFunction.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_solver/TasksQPSolver.h>

#include <mc_rbdyn/rpy_utils.h>

#include <mc_rbdyn/hat.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/deprecated.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

static inline mc_rtc::void_ptr_caster<tasks::qp::SurfaceTransformTask> tasks_error{};
static inline mc_rtc::void_ptr_caster<mc_tvm::TransformFunction> tvm_error{};

TransformTask::TransformTask(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: TrajectoryTaskGeneric(frame.robot().robots(), frame.robot().robotIndex(), stiffness, weight), frame_(frame)
{
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::SurfaceTransformTask>(robots.mbs(), static_cast<int>(rIndex), frame.body(),
                                                                frame.position(), frame.X_b_f());
      break;
    case Backend::TVM:
      finalize<Backend::TVM, mc_tvm::TransformFunction>(frame);
      break;
    default:
      mc_rtc::log::error_and_throw("[TransformTask] Not implemented for solver backend: {}", backend_);
  }

  type_ = "transform";
  name_ = "transform_" + frame.robot().name() + "_" + frame.name();
}

TransformTask::TransformTask(const std::string & surfaceName,
                             const mc_rbdyn::Robots & robots,
                             unsigned int robotIndex,
                             double stiffness,
                             double weight)
: TransformTask(robots.robot(robotIndex).frame(surfaceName), stiffness, weight)
{
}

void TransformTask::reset()
{
  TrajectoryTaskGeneric::reset();
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->target(frame_->position());
      break;
    case Backend::TVM:
      tvm_error(errorT)->reset();
      break;
    default:
      break;
  }
}

/*! \brief Load parameters from a Configuration object */
void TransformTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  // Current surface position
  sva::PTransformd X_0_t = surfacePose();

  // Apply global world transformations first
  if(config.has("targetSurface"))
  {
    const auto & c = config("targetSurface");
    const auto robotIndex = robotIndexFromConfig(c, solver.robots(), name());
    targetSurface(robotIndex, c("surface"),
                  {c("offset_rotation", Eigen::Matrix3d::Identity().eval()),
                   c("offset_translation", Eigen::Vector3d::Zero().eval())});
    X_0_t = this->target();
  }
  else if(config.has("targetFrame"))
  {
    const auto & c = config("targetFrame");
    const auto robotIndex = robotIndexFromConfig(c, solver.robots(), name());
    target(robots.robot(robotIndex).frame(c("frame")), c("offset", sva::PTransformd::Identity()));
  }
  else if(config.has("target")) { X_0_t = config("target"); }
  else if(config.has("relative"))
  {
    const auto & robot = robotFromConfig(config("relative"), solver.robots(), name());
    std::string s1 = config("relative")("s1");
    std::string s2 = config("relative")("s2");
    sva::PTransformd target = config("relative")("target");
    auto X_0_s1 = robot.surfacePose(s1);
    auto X_0_s2 = robot.surfacePose(s2);
    auto X_s1_s2 = X_0_s2 * X_0_s1.inv();
    X_s1_s2.translation() = X_s1_s2.translation() / 2;
    auto X_0_relative = X_s1_s2 * X_0_s1;
    this->target(target * X_0_relative);
  }
  else
  {
    if(config.has("targetPosition")) { X_0_t.translation() = config("targetPosition"); }
    if(config.has("targetRotation")) { X_0_t.rotation() = config("targetRotation"); }
  }

  if(config.has("moveWorld"))
  {
    sva::PTransformd move = config("moveWorld");
    X_0_t = X_0_t * move;
  }

  if(config.has("move"))
  {
    sva::PTransformd move = config("move");
    X_0_t = move * X_0_t;
  }

  if(config.has("overwriteRPY"))
  {
    // Only modify the specified DoF of the rotation
    mc_rtc::overwriteRotationRPY(config("overwriteRPY"), X_0_t.rotation());
  }

  this->target(X_0_t);

  TrajectoryBase::load(solver, config);
}

sva::PTransformd TransformTask::target() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(errorT)->target();
    case Backend::TVM:
      return tvm_error(errorT)->pose();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void TransformTask::target(const sva::PTransformd & worldPose)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->target(worldPose);
      break;
    case Backend::TVM:
      tvm_error(errorT)->pose(worldPose);
      break;
    default:
      break;
  }
}

void TransformTask::targetVel(const sva::MotionVecd & worldVec)
{
  auto X_0_f = frame_->position();
  auto velB = X_0_f * worldVec;
  refVelB(velB);
}

void TransformTask::targetSurface(unsigned int robotIndex,
                                  const std::string & surfaceName,
                                  const sva::PTransformd & offset)
{
  target(robots.robot(robotIndex).frame(surfaceName), offset);
}

void TransformTask::targetFrame(const mc_rbdyn::Frame & targetFrame, const sva::PTransformd & offset)
{
  target(targetFrame, offset);
}

void TransformTask::targetFrameVelocity(const mc_rbdyn::Frame & targetFrame, const sva::PTransformd & offset)
{
  auto vel = targetFrame.velocity();
  auto X_0_f = targetFrame.position();
  vel.linear() += -mc_rbdyn::hat(X_0_f.rotation().transpose() * offset.translation()) * vel.angular();
  targetVel(vel);
}

void TransformTask::target(const mc_rbdyn::Frame & frame, const sva::PTransformd & offset)
{
  target(offset * frame.position());
}

void TransformTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_pose", this, [this]() { return frame_->position(); });
  logger.addLogEntry(name_ + "_target_pose", this, [this]() { return target(); });
}

std::function<bool(const mc_tasks::MetaTask &, std::string &)> TransformTask::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{
  if(config.has("wrench"))
  {
    if(!frame_->hasForceSensor())
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("[{}] Attempted to use \"wrench\" as completion criteria but "
                                                          "frame \"{}\" is not attached to a force sensor",
                                                          name(), frame_->name());
    }
    sva::ForceVecd target_w = config("wrench");
    Eigen::Vector6d target = target_w.vector();
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    for(int i = 0; i < 6; ++i)
    {
      if(std::isnan(target(i)))
      {
        dof(i) = 0.;
        target(i) = 0.;
      }
      else if(target(i) < 0) { dof(i) = -1.; }
    }
    return [dof, target](const mc_tasks::MetaTask & t, std::string & out)
    {
      const auto & self = static_cast<const mc_tasks::TransformTask &>(t);
      Eigen::Vector6d w = self.robots.robot(self.rIndex).surfaceWrench(self.surface()).vector();
      for(int i = 0; i < 6; ++i)
      {
        if(dof(i) * fabs(w(i)) < target(i)) { return false; }
      }
      out += "wrench";
      return true;
    };
  }
  return MetaTask::buildCompletionCriteria(dt, config);
}

void TransformTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Transform(
                     "pos_target", [this]() { return this->target(); },
                     [this](const sva::PTransformd & pos) { this->target(pos); }),
                 mc_rtc::gui::Transform("pos", [this]() { return frame_->position(); }));
}

} // namespace mc_tasks

namespace
{

static mc_tasks::MetaTaskPtr loadTransformTask(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "transform");
  const auto & robot = solver.robots().robot(robotIndex);
  const auto & frame = [&]() -> const mc_rbdyn::RobotFrame &
  {
    if(config.has("surface"))
    {
      mc_rtc::log::deprecated("TransformTask", "surface", "frame");
      return robot.frame(config("surface"));
    }
    else { return robot.frame(config("frame")); }
  }();
  auto t = std::make_shared<mc_tasks::TransformTask>(frame);
  t->load(solver, config);
  return t;
}

static auto reg_dep = mc_tasks::MetaTaskLoader::register_load_function(
    "surfaceTransform",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      mc_rtc::log::deprecated("TaskLoading", "surfaceTransform", "transform");
      return loadTransformTask(solver, config);
    });
static auto reg = mc_tasks::MetaTaskLoader::register_load_function("transform", &loadTransformTask);

} // namespace
