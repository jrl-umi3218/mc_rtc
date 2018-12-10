#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

AddRemoveContactTask::AddRemoveContactTask(mc_rbdyn::Robots & robots,
                                           std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                                           mc_rbdyn::Contact & contact,
                                           double direction,
                                           double _speed,
                                           double stiffness,
                                           double weight,
                                           Eigen::Vector3d * userT_0_s)
: robots(robots), robot(robots.robot(contact.r1Index())), env(robots.robot(contact.r2Index())),
  constSpeedConstr(constSpeedConstr), robotSurf(contact.r1Surface()),
  robotBodyIndex(robot.bodyIndexByName(robotSurf->bodyName())), targetTf(contact.X_0_r1s(robots)),
  bodyId(robotSurf->bodyName()), dofMat(Eigen::MatrixXd::Zero(5, 6)), speedMat(Eigen::VectorXd::Zero(5)),
  stiffness_(stiffness), weight_(weight)
{
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
  linVelTask.reset(new tasks::qp::LinVelocityTask(robots.mbs(), 0, robotSurf->bodyName(), targetSpeed,
                                                  robotSurf->X_b_s().translation())),
      linVelTaskPid.reset(new tasks::qp::PIDTask(robots.mbs(), 0, linVelTask.get(), stiffness, 0, 0, 0));
  linVelTaskPid->error(velError());
  linVelTaskPid->errorI(Eigen::Vector3d(0, 0, 0));
  linVelTaskPid->errorD(Eigen::Vector3d(0, 0, 0));
  targetVelWeight = weight;
}

AddRemoveContactTask::AddRemoveContactTask(mc_solver::QPSolver & solver,
                                           mc_rbdyn::Contact & contact,
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
}

void AddRemoveContactTask::speed(double s)
{
  speed_ = s;
  targetSpeed = direction_ * normal * speed_;
}

Eigen::Vector3d AddRemoveContactTask::velError()
{
  Eigen::Vector3d T_b_s = robotSurf->X_b_s().translation();
  Eigen::Matrix3d E_0_b = robot.mbc().bodyPosW[robotBodyIndex].rotation();

  sva::PTransformd pts(E_0_b.transpose(), T_b_s);
  sva::MotionVecd surfVelB = pts * robot.mbc().bodyVelB[robotBodyIndex];
  return targetSpeed - surfVelB.linear();
}

void AddRemoveContactTask::addToSolver(mc_solver::QPSolver & solver)
{
  solver.addTask(linVelTaskPid.get());
  if(manageConstraint)
  {
    solver.addConstraintSet(*constSpeedConstr);
  }
  constSpeedConstr->addBoundedSpeed(solver, bodyId, robotSurf->X_b_s().translation(), dofMat, speedMat);
}

void AddRemoveContactTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  solver.removeTask(linVelTaskPid.get());
  if(manageConstraint)
  {
    solver.removeConstraintSet(*constSpeedConstr);
  }
  constSpeedConstr->removeBoundedSpeed(solver, bodyId);
}

void AddRemoveContactTask::update()
{
  linVelTaskPid->error(velError());
  linVelTaskPid->weight(std::min(linVelTaskPid->weight() + 0.5, targetVelWeight));
}

void AddRemoveContactTask::dimWeight(const Eigen::VectorXd & dimW)
{
  linVelTaskPid->dimWeight(dimW);
}

Eigen::VectorXd AddRemoveContactTask::dimWeight() const
{
  return linVelTaskPid->dimWeight();
}

Eigen::VectorXd AddRemoveContactTask::eval() const
{
  return linVelTask->eval();
}

Eigen::VectorXd AddRemoveContactTask::speed() const
{
  return linVelTask->speed();
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
  return std::make_shared<T>(solver, contact, config("speed"), config("stiffness"), config("weight"), userT_0_s);
}

static bool registered =
    mc_tasks::MetaTaskLoader::register_load_function("addContact",
                                                     &load_add_remove_contact_task<mc_tasks::AddContactTask>)
    && mc_tasks::MetaTaskLoader::register_load_function("removeContact",
                                                        &load_add_remove_contact_task<mc_tasks::RemoveContactTask>);

} // namespace
