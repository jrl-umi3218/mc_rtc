/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/AddRemoveContactTask.h>

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_tvm/FrameVelocity.h>

#include <mc_solver/TasksQPSolver.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/logging.h>

namespace mc_tasks
{

namespace details
{

struct TasksImpl
{
  std::unique_ptr<tasks::qp::LinVelocityTask> linVelTask;
  std::unique_ptr<tasks::qp::PIDTask> linVelTaskPid;
};

struct TVMImpl : public mc_tasks::TrajectoryTaskGeneric
{
  TVMImpl(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
  : mc_tasks::TrajectoryTaskGeneric(frame, stiffness, weight)
  {
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    dof.head(3).setZero();
    finalize<Backend::TVM, mc_tvm::FrameVelocity>(frame, dof);
  }
};

mc_rtc::void_ptr initialize(MetaTask::Backend backend,
                            const mc_rbdyn::RobotFrame & frame,
                            double stiffness,
                            double weight)
{
  switch(backend)
  {
    case MetaTask::Backend::Tasks:
      return mc_rtc::make_void_ptr<TasksImpl>();
    case MetaTask::Backend::TVM:
      return mc_rtc::make_void_ptr<TVMImpl>(frame, stiffness, weight);
    default:
      mc_rtc::log::error_and_throw("[AddRemoveContactTask] Not implemented for solver backend: {}", backend);
  }
}

} // namespace details

static inline mc_rtc::void_ptr_caster<details::TasksImpl> tasks_impl{};
static inline mc_rtc::void_ptr_caster<details::TVMImpl> tvm_impl{};

AddRemoveContactTask::AddRemoveContactTask(const mc_rbdyn::Robots & robots,
                                           std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                                           const mc_rbdyn::Contact & contact,
                                           double direction,
                                           double _speed,
                                           double stiffness,
                                           double weight,
                                           Eigen::Vector3d * userT_0_s)
: robots(robots), robotIndex(contact.r1Index()), envIndex(contact.r2Index()), constSpeedConstr(constSpeedConstr),
  robotSurf(contact.r1Surface()), robotBodyIndex(robots.robot(robotIndex).bodyIndexByName(robotSurf->bodyName())),
  targetTf(contact.X_0_r1s(robots)), bodyId(robotSurf->bodyName()), dofMat(Eigen::MatrixXd::Zero(6, 6)),
  speedMat(Eigen::VectorXd::Zero(6)), stiffness_(stiffness), weight_(weight),
  impl_(details::initialize(backend_, robots.robot(robotIndex).frame(contact.r1Surface()->name()), stiffness, weight))
{
  const auto & robot = robots.robot(robotIndex);
  const auto & env = robots.robot(envIndex);
  type_ = std::string(direction > 0 ? "removeContact" : "addContact");
  name_ = std::string(direction > 0 ? "remove" : "add") + "_contact_" + robot.name() + "_" + contact.r1Surface()->name()
          + "_" + env.name() + "_" + contact.r2Surface()->name();
  for(int i = 0; i < 5; ++i)
  {
    dofMat(i, i) = 1;
  }
  normal = targetTf.rotation().row(2);

  if(userT_0_s)
  {
    normal = (*userT_0_s - targetTf.translation()).normalized();
    auto normalBody = robot.mbc().bodyPosW[robotBodyIndex].rotation() * normal;
    Eigen::Vector3d v1(normalBody.y(), -normalBody.x(), 0);
    Eigen::Vector3d v2(-normalBody.z(), 0, normalBody.x());
    Eigen::Vector3d T = (v1 + v2).normalized();
    Eigen::Vector3d B = normalBody.cross(T);
    for(int i = 3; i < 6; ++i)
    {
      dofMat(3, i) = T[i - 3];
      dofMat(4, i) = B[i - 3];
    }
  }

  direction_ = direction;
  this->speed_ = _speed;
  targetSpeed = direction * normal * speed_;
  targetVelWeight = weight;

  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto impl = tasks_impl(impl_);
      impl->linVelTask.reset(new tasks::qp::LinVelocityTask(robots.mbs(), 0, robotSurf->bodyName(), targetSpeed,
                                                            robotSurf->X_b_s().translation())),
          impl->linVelTaskPid.reset(
              new tasks::qp::PIDTask(robots.mbs(), 0, impl->linVelTask.get(), stiffness, 0, 0, 0));
      impl->linVelTaskPid->error(velError());
      impl->linVelTaskPid->errorI(Eigen::Vector3d(0, 0, 0));
      impl->linVelTaskPid->errorD(Eigen::Vector3d(0, 0, 0));
      break;
    }
    case Backend::TVM:
    {
      Eigen::Vector6d refVel = Eigen::Vector6d::Zero();
      refVel.tail(3) = targetSpeed;
      tvm_impl(impl_)->refVel(refVel);
      break;
    }
    default:
      break;
  }
}

AddRemoveContactTask::AddRemoveContactTask(const mc_solver::QPSolver & solver,
                                           const mc_rbdyn::Contact & contact,
                                           double direction,
                                           double _speed,
                                           double stiffness,
                                           double weight,
                                           Eigen::Vector3d * userT_0_s)
: AddRemoveContactTask(solver.robots(), nullptr, contact, direction, _speed, stiffness, weight, userT_0_s)
{
  manageConstraint = true;
  constSpeedConstr = std::make_shared<mc_solver::BoundedSpeedConstr>(solver.robots(), contact.r1Index(), solver.dt());
}

void AddRemoveContactTask::direction(double direction)
{
  direction_ = direction;
  targetSpeed = direction_ * normal * speed_;
  if(backend_ == Backend::TVM)
  {
    Eigen::Vector6d refVel = Eigen::Vector6d::Zero();
    refVel.tail(3) = targetSpeed;
    tvm_impl(impl_)->refVel(refVel);
  }
}

void AddRemoveContactTask::speed(double s)
{
  speed_ = s;
  targetSpeed = direction_ * normal * speed_;
  if(backend_ == Backend::TVM)
  {
    Eigen::Vector6d refVel = Eigen::Vector6d::Zero();
    refVel.tail(3) = targetSpeed;
    tvm_impl(impl_)->refVel(refVel);
  }
}

Eigen::Vector3d AddRemoveContactTask::velError()
{
  const auto & robot = robots.robot(robotIndex);
  Eigen::Vector3d T_b_s = robotSurf->X_b_s().translation();
  Eigen::Matrix3d E_0_b = robot.mbc().bodyPosW[robotBodyIndex].rotation();

  sva::PTransformd pts(E_0_b.transpose(), T_b_s);
  sva::MotionVecd surfVelB = pts * robot.mbc().bodyVelB[robotBodyIndex];
  return targetSpeed - surfVelB.linear();
}

void AddRemoveContactTask::addToSolver(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_solver(solver).addTask(tasks_impl(impl_)->linVelTaskPid.get());
      break;
    case Backend::TVM:
      MetaTask::addToSolver(*tvm_impl(impl_), solver);
      break;
    default:
      break;
  }
  if(manageConstraint)
  {
    solver.addConstraintSet(*constSpeedConstr);
  }
  constSpeedConstr->addBoundedSpeed(solver, bodyId, robotSurf->X_b_s().translation(), dofMat, speedMat);
}

void AddRemoveContactTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_solver(solver).removeTask(tasks_impl(impl_)->linVelTaskPid.get());
      break;
    case Backend::TVM:
      MetaTask::removeFromSolver(*tvm_impl(impl_), solver);
      break;
    default:
      break;
  }
  if(manageConstraint)
  {
    solver.removeConstraintSet(*constSpeedConstr);
  }
  constSpeedConstr->removeBoundedSpeed(solver, bodyId);
}

void AddRemoveContactTask::update(mc_solver::QPSolver &)
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto linVelTaskPid = tasks_impl(impl_)->linVelTaskPid.get();
      linVelTaskPid->error(velError());
      linVelTaskPid->weight(std::min(linVelTaskPid->weight() + 0.5, targetVelWeight));
      break;
    }
    case Backend::TVM:
    {
      auto impl = tvm_impl(impl_);
      impl->weight(std::min(impl->weight() + 0.5, targetVelWeight));
      break;
    }
    default:
      break;
  }
}

void AddRemoveContactTask::dimWeight(const Eigen::VectorXd & dimW)
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto linVelTaskPid = tasks_impl(impl_)->linVelTaskPid.get();
      linVelTaskPid->dimWeight(dimW);
      break;
    }
    case Backend::TVM:
    {
      auto impl = tvm_impl(impl_);
      impl->dimWeight(dimW);
      break;
    }
    default:
      break;
  }
}

Eigen::VectorXd AddRemoveContactTask::dimWeight() const
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto linVelTaskPid = tasks_impl(impl_)->linVelTaskPid.get();
      return linVelTaskPid->dimWeight();
    }
    case Backend::TVM:
    {
      auto impl = tvm_impl(impl_);
      return impl->dimWeight();
    }
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

Eigen::VectorXd AddRemoveContactTask::eval() const
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto linVelTask = tasks_impl(impl_)->linVelTask.get();
      return linVelTask->eval();
    }
    case Backend::TVM:
    {
      auto impl = tvm_impl(impl_);
      return impl->eval();
    }
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

Eigen::VectorXd AddRemoveContactTask::speed() const
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      auto linVelTask = tasks_impl(impl_)->linVelTask.get();
      return linVelTask->speed();
    }
    case Backend::TVM:
    {
      auto impl = tvm_impl(impl_);
      return impl->speed();
    }
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

AddContactTask::AddContactTask(mc_rbdyn::Robots & robots,
                               std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                               mc_rbdyn::Contact & contact,
                               double speed,
                               double stiffness,
                               double weight,
                               Eigen::Vector3d * userT_0_s)
: AddRemoveContactTask(robots, constSpeedConstr, contact, -1.0, speed, stiffness, weight, userT_0_s)
{
}

AddContactTask::AddContactTask(mc_solver::QPSolver & solver,
                               mc_rbdyn::Contact & contact,
                               double speed,
                               double stiffness,
                               double weight,
                               Eigen::Vector3d * userT_0_s)
: AddRemoveContactTask(solver, contact, -1.0, speed, stiffness, weight, userT_0_s)
{
}

RemoveContactTask::RemoveContactTask(mc_rbdyn::Robots & robots,
                                     std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                                     mc_rbdyn::Contact & contact,
                                     double speed,
                                     double stiffness,
                                     double weight,
                                     Eigen::Vector3d * userT_0_s)
: AddRemoveContactTask(robots, constSpeedConstr, contact, 1.0, speed, stiffness, weight, userT_0_s)
{
}

RemoveContactTask::RemoveContactTask(mc_solver::QPSolver & solver,
                                     mc_rbdyn::Contact & contact,
                                     double speed,
                                     double stiffness,
                                     double weight,
                                     Eigen::Vector3d * userT_0_s)
: AddRemoveContactTask(solver, contact, 1.0, speed, stiffness, weight, userT_0_s)
{
}

} // namespace mc_tasks

namespace
{

template<typename T>
mc_tasks::MetaTaskPtr load_add_remove_contact_task(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  auto contact = mc_rbdyn::Contact::load(solver.robots(), config("contact"));
  Eigen::Vector3d T_0_s;
  Eigen::Vector3d * userT_0_s = nullptr;
  if(config.has("T_0_s"))
  {
    T_0_s = config("T_0_s");
    userT_0_s = &T_0_s;
  }
  return std::make_shared<T>(solver, contact, config("speed", 0.01), config("stiffness", 2.), config("weight", 1000.),
                             userT_0_s);
}

static auto ac_registered =
    mc_tasks::MetaTaskLoader::register_load_function("addContact",
                                                     &load_add_remove_contact_task<mc_tasks::AddContactTask>);
static auto rc_registered =
    mc_tasks::MetaTaskLoader::register_load_function("removeContact",
                                                     &load_add_remove_contact_task<mc_tasks::RemoveContactTask>);

} // namespace
