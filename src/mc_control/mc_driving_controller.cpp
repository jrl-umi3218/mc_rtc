#include <mc_control/mc_driving_controller.h>
#include <mc_robots/polaris_ranger.h>
#include <mc_rbdyn/robot.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

MCDrivingController::MCDrivingController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule> >& env_modules)
  : MCMRQPController(env_modules),
    ef_task("RARM_LINK6", robots(), 0),
    polarisKinematicsConstraint(robots(), 1, timeStep, false,
        {0.1, 0.01, 0.5}, 0.5)
{
  mrqpsolver->addConstraintSet(hrp2contactConstraint);
  mrqpsolver->addConstraintSet(hrp2kinematicsConstraint);
  mrqpsolver->addConstraintSet(polarisKinematicsConstraint);
  //mrqpsolver->addConstraintSet(hrp2selfCollisionConstraint);
  mrqpsolver->solver.addTask(hrp2postureTask.get());
  robots().envIndex = 2;

  mc_rbdyn::Robot& polaris = robots().robots[1];

  robot().mbc->q[0] = {0.8018680589369662, 0.09936561148509283, -0.06541812773434774, 0.5855378381237102, -0.3421374123035909, -0.0002850914593993392, 0.8847053544605464};

  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));

  mrqpsolver->setContacts({mc_rbdyn::MRContact(robots().robotIndex,
                           1,
                           robot().surfaces.at("Butthock"),
                           polaris.surfaces.at("left_seat_deformed")),
                           mc_rbdyn::MRContact(robots().robotIndex,
                           1,
                           robot().surfaces.at("LowerBack"),
                           polaris.surfaces.at("left_back"))});

  ef_task.addToSolver(mrqpsolver->solver);
  int steer_i = polaris.bodyIndexByName("steering_wheel");
  ef_task.set_ef_pose(polaris.mbc->bodyPosW[steer_i]);

  std::cout << "MCDrivingController init done" << std::endl;
}

bool MCDrivingController::run()
{
  bool success = MCMRQPController::run();
  return success;
}

void MCDrivingController::reset(const ControllerResetData & reset_data)
{
  mc_rbdyn::Robot& polaris = robots().robots[1];
  robot().mbc->zero(*(robot().mb));
  robot().mbc->q = reset_data.q;
  robot().mbc->q[0] = {0.8018680589369662, 0.09936561148509283, -0.06541812773434774, 0.5855378381237102, -0.3421374123035909, -0.0002850914593993392, 0.8847053544605464};
  hrp2postureTask->posture(robot().mbc->q);
  rbd::forwardKinematics(*(robot().mb), *(robot().mbc));
  rbd::forwardVelocity(*(robot().mb), *(robot().mbc));
  mrqpsolver->setContacts({mc_rbdyn::MRContact(robots().robotIndex,
                           1,
                           robot().surfaces.at("Butthock"),
                           polaris.surfaces.at("left_seat_deformed")),
                           mc_rbdyn::MRContact(robots().robotIndex,
                           1,
                           robot().surfaces.at("LowerBack"),
                           polaris.surfaces.at("left_back"))});
}

bool MCDrivingController::changeWheelAngle(double theta)
{
  std::cout << "Wheel angle " << theta << std::endl;
  return true;
}

bool MCDrivingController::changeAnkleAngle(double theta)
{
  int ankle_i = robot().jointIndexByName("RLEG_JOINT4");
  auto p = hrp2postureTask->posture();
  p[ankle_i][0] = theta;
  hrp2postureTask->posture(p);
  std::cout << "Ankle angle set to : " << theta << std::endl;
  return true;
}

}
