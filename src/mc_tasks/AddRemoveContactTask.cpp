#include <mc_tasks/AddRemoveContactTask.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Surface.h>

namespace mc_tasks
{

AddRemoveContactTask::AddRemoveContactTask(mc_rbdyn::Robots & robots, std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr, mc_rbdyn::Contact & contact,
                       double direction, const mc_rbdyn::StanceConfig & config,
                       Eigen::Vector3d * userT_0_s)
: robots(robots), robot(robots.robot()), env(robots.env()),
  constSpeedConstr(constSpeedConstr), robotSurf(contact.r1Surface()),
  robotBodyIndex(robot.bodyIndexByName(robotSurf->bodyName())),
  robotBodyId(robot.bodyIdByName(robotSurf->bodyName())),
  targetTf(contact.X_0_r1s(robots)),
  bodyId(robotSurf->bodyName()),
  dofMat(Eigen::MatrixXd::Zero(5,6)), speedMat(Eigen::VectorXd::Zero(5))
{
  for(int i = 0; i < 5; ++i)
  {
    dofMat(i,i) = 1;
  }
  normal = targetTf.rotation().row(2);

  if(userT_0_s)
  {
    normal = (*userT_0_s - targetTf.translation()).normalized();
    auto normalBody = robot.mbc().bodyPosW[robotBodyIndex].rotation()*normal;
    Eigen::Vector3d v1(normalBody.y(), -normalBody.x(), 0);
    Eigen::Vector3d v2(-normalBody.z(), 0, normalBody.x());
    Eigen::Vector3d T = (v1 + v2).normalized();
    Eigen::Vector3d B = normalBody.cross(T);
    for(int i = 3; i < 6; ++i)
    {
      dofMat(3, i) = T[i-3];
      dofMat(4, i) = B[i-3];
    }
  }

  speed = config.contactTask.linVel.speed;
  targetSpeed = direction*normal*speed;
  linVelTask.reset(new tasks::qp::LinVelocityTask(robots.mbs(), 0, robotBodyId, targetSpeed, robotSurf->X_b_s().translation())),
  linVelTaskPid.reset(new tasks::qp::PIDTask(robots.mbs(), 0, linVelTask.get(), config.contactTask.linVel.stiffness, 0, 0, 0));
  linVelTaskPid->error(velError());
  linVelTaskPid->errorI(Eigen::Vector3d(0, 0, 0));
  linVelTaskPid->errorD(Eigen::Vector3d(0, 0, 0));
  targetVelWeight = config.contactTask.linVel.weight;
}

void AddRemoveContactTask::direction(double direction)
{
  linVelTask->velocity(direction*normal*speed);
}

Eigen::Vector3d AddRemoveContactTask::velError()
{
  Eigen::Vector3d T_b_s = robotSurf->X_b_s().translation();
  Eigen::Matrix3d E_0_b = robot.mbc().bodyPosW[robotBodyIndex].rotation();
  sva::PTransformd pts(E_0_b.transpose(), T_b_s);
  sva::MotionVecd surfVelB = pts*robot.mbc().bodyVelB[robotBodyIndex];
  return targetSpeed - surfVelB.linear();
}


void AddRemoveContactTask::addToSolver(mc_solver::QPSolver & solver)
{
  solver.addTask(linVelTaskPid.get());
  constSpeedConstr->addBoundedSpeed(solver, bodyId, robotSurf->X_b_s().translation(), dofMat, speedMat);
}

void AddRemoveContactTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  solver.removeTask(linVelTaskPid.get());
  constSpeedConstr->removeBoundedSpeed(solver, bodyId);
}

void AddRemoveContactTask::update()
{
  linVelTaskPid->error(velError());
  linVelTaskPid->weight(std::min(linVelTaskPid->weight() + 0.5, targetVelWeight));
}

AddContactTask::AddContactTask(mc_rbdyn::Robots & robots, std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                               mc_rbdyn::Contact & contact,
                               const mc_rbdyn::StanceConfig & config,
                               Eigen::Vector3d * userT_0_s)
: AddRemoveContactTask(robots, constSpeedConstr, contact, -1.0, config, userT_0_s)
{
}

RemoveContactTask::RemoveContactTask(mc_rbdyn::Robots & robots, std::shared_ptr<mc_solver::BoundedSpeedConstr> constSpeedConstr,
                               mc_rbdyn::Contact & contact,
                               const mc_rbdyn::StanceConfig & config,
                               Eigen::Vector3d * userT_0_s)
: AddRemoveContactTask(robots, constSpeedConstr, contact, 1.0, config, userT_0_s)
{
}

}
