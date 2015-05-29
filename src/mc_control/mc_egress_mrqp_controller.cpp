#include <mc_control/mc_egress_mrqp_controller.h>
#include <mc_robots/polaris_ranger_egress.h>
#include <mc_rbdyn/robot.h>

#include <Tasks/QPContactConstr.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

MCEgressMRQPController::MCEgressMRQPController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule> >& env_modules)
  : MCMRQPController(env_modules),
    polarisKinematicsConstraint(robots(), 1, timeStep, false,
        {0.1, 0.01, 0.01}, 0.5),
    egressContacts(),
    collsConstraint(robots(), 0, 1, timeStep),
    lazy_theta(0)
{
  mrqpsolver->addConstraintSet(hrp2contactConstraint);
  mrqpsolver->addConstraintSet(hrp2kinematicsConstraint);
  mrqpsolver->addConstraintSet(polarisKinematicsConstraint);
  mrqpsolver->addConstraintSet(hrp2selfCollisionConstraint);
  mrqpsolver->addConstraintSet(collsConstraint);

  mrqpsolver->solver.addTask(hrp2postureTask.get());
  robots().envIndex = 2;

  mc_rbdyn::Robot& polaris = robots().robots[1];

  //robot().mbc->q[0] = {0.8018680589369662, 0.09936561148509283, -0.06541812773434774, 0.5855378381237102, -0.3421374123035909, -0.0002850914593993392, 0.8847053544605464};
  robot().mbc->q[0] = {1, 0, 0, 0, 0, 0, 0.76};

  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  egressContacts.emplace_back(robots().robotIndex, 1,
                           robot().surfaces.at("Butthock"),
                           polaris.surfaces.at("lazy_seat"));
  egressContacts.emplace_back(robots().robotIndex, 1,
                           robot().surfaces.at("LFullSole"),
                           polaris.surfaces.at("exit_platform"));
  egressContacts.emplace_back(robots().robotIndex, 1,
                           robot().surfaces.at("LeftThight"),
                           polaris.surfaces.at("lazy_seat"));
  egressContacts.emplace_back(robots().robotIndex, 1,
                           robot().surfaces.at("RightThight"),
                           polaris.surfaces.at("lazy_seat"));
  egressContacts.emplace_back(robots().robotIndex, 1,
                           robot().surfaces.at("RightGripper"),
                           polaris.surfaces.at("bar_wheel"));
  //egressContacts.emplace_back(robots().robotIndex, 1,
  //                         robot().surfaces.at("LowerBack"),
  //                         polaris.surfaces.at("left_back"));
  mrqpsolver->setContacts(egressContacts);
  //collsConstraint.addCollision(robots(),
  //  mc_solver::Collision("CHEST_LINK1", "seat_back", 0.4, 0.25, 0.0)
  //);

  //ef_task.addToSolver(mrqpsolver->solver);
  //ef_task.removeFromSolver(mrqpsolver->solver);

  polarisPostureTask = std::shared_ptr<tasks::qp::PostureTask>(new tasks::qp::PostureTask(mrqpsolver->robots.mbs, 1, mrqpsolver->robots.robots[1].mbc->q, 0.5, 100));

  std::cout << "MCEgressMRQPController init done" << std::endl;
}

bool MCEgressMRQPController::run()
{
  bool success = MCMRQPController::run();
  lazy_theta += 1;
  int lazy_i = robots().robots[1].jointIndexByName("lazy_susan");
  auto p = polarisPostureTask->posture();
  p[lazy_i][0] = M_PI/2;
  polarisPostureTask->posture(p);
  return success;
}

void MCEgressMRQPController::reset(const ControllerResetData & reset_data)
{
  MCMRQPController::reset(reset_data);
  std::cout << "Enter egress reset" << std::endl;
  mc_rbdyn::Robot& polaris = robots().robots[1];
  robot().mbc->zero(*(robot().mb));
  robot().mbc->q = reset_data.q;
  //robot().mbc->q[0] = {0.8018680589369662, 0.09936561148509283, -0.06541812773434774, 0.5855378381237102, -0.3421374123035909, -0.0002850914593993392, 0.8847053544605464};
  robot().mbc->q[0] = {1, 0, 0, 0, 0, 0, 0.76};
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));
  hrp2postureTask->posture(robot().mbc->q);

  resetBasePose();

  mrqpsolver->solver.addTask(polarisPostureTask.get());

  mrqpsolver->setContacts(egressContacts);

  mrqpsolver->solver.updateTasksNrVars(robots().mbs);
  mrqpsolver->solver.updateConstrsNrVars(robots().mbs);
  mrqpsolver->solver.updateConstrSize();

  std::cout << "End egress reset" << std::endl;
}

void MCEgressMRQPController::resetBasePose()
{
  mc_rbdyn::Robot& polaris = robots().robots[1];
  //Reset freeflyer, compute its position frow wheel and re-set it
  robot().mbc->q[0] = {1., 0., 0., 0., 0., 0., 0.};
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  int steer_i = polaris.bodyIndexByName("steering_wheel");
  sva::PTransformd X_0_w = polaris.mbc->bodyPosW[steer_i];
  auto gripperSurface = robot().surfaces.at("RightGripper");
  sva::PTransformd X_0_s = gripperSurface->X_0_s(robot(), *(robot().mbc));
  sva::PTransformd graspOffset(sva::RotX(-M_PI/2), Eigen::Vector3d(0., 0., 0.));
  sva::PTransformd X_0_base = X_0_s.inv()*(graspOffset*X_0_w);
  //sva::PTransformd X_0_base = X_0_s.inv()*X_0_w;
  X_0_w = polaris.surfaces.at("exit_platform")->X_0_s(polaris);
  X_0_s = robot().surfaces.at("LFullSole")->X_0_s(robot());
  X_0_base = X_0_s.inv()*X_0_w;

  const auto quat = Eigen::Quaterniond(X_0_base.rotation()).inverse();
  const Eigen::Vector3d trans(X_0_base.translation());
  std::vector<double> baseQ = {quat.w(), quat.x(), quat.y(), quat.z(),
                               trans.x(), trans.y(), trans.z()};

  robot().mbc->q[0] = baseQ;
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));
}

}
