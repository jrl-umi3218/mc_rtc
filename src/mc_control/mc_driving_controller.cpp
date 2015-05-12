#include <mc_control/mc_driving_controller.h>
#include <mc_robots/polaris_ranger.h>

#include <mc_rbdyn/robot.h>

namespace mc_control
{

MCDrivingController::MCDrivingController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule> >& env_modules)
  : MCMRQPController(env_modules),
    ef_task("RARM_LINK6", robots(), 0),
    polarisKinematicsConstraint(robots(), robots().envIndex, timeStep, false,
        {0.1, 0.01, 0.5}, 0.5)
{
  mrqpsolver->addConstraintSet(hrp2contactConstraint);
  mrqpsolver->addConstraintSet(hrp2kinematicsConstraint);
  mrqpsolver->addConstraintSet(polarisKinematicsConstraint);
  mrqpsolver->addConstraintSet(hrp2selfCollisionConstraint);
  mrqpsolver->solver.addTask(hrp2postureTask.get());
  robots().envIndex = 1;

  mc_rbdyn::Robot& polaris = env();

  mrqpsolver->setContacts({mc_rbdyn::MRContact(0, 1, robot().surfaces.at("Butthock"),
        env().surfaces.at("left_seat_deformed"))});

  ef_task.addToSolver(mrqpsolver->solver);
  int steer_i = polaris.bodyIndexByName("steering_wheel");
  //ef_task.set_ef_pose(polaris.mbc->bodyPosW[steer_i]);
  ef_task.set_ef_pose(sva::PTransformd(Eigen::Vector3d(1, 1, 1)));

  std::cout << "MCDrivingController init done" << std::endl;
  std::cout << env().mb->joint(0).type() << std::endl;
  std::cout << env().mb->joint(1).name() << std::endl;
  std::cout << (env().mb->joint(0).type() == rbd::Joint::Fixed) << std::endl;
  std::cout << robots().envIndex << " " << robots().robotIndex << "Base size " << env().mbc->q[0].size() << std::endl;
  std::cout << "EF : " << ef_task.positionTask->eval().norm() << std::endl;
}

bool MCDrivingController::run()
{
  bool success = MCMRQPController::run();
  //int ankle_i = robot().jointIndexByName("RLEG_JOINT4");
  //std::cout << robot().mbc->q[ankle_i][0] << std::endl;
  //std::cout << ef_task.positionTask->eval().norm() << std::endl;
  std::cout << robot().mbc->q[0][3] << robot().mbc->q[0][4] << robot().mbc->q[0][5] << std::endl;
  return success;
}

bool MCDrivingController::changeWheelAngle(double theta)
{
  std::cout << "Wheel angle " << theta << std::endl;
  return true;
}

bool MCDrivingController::changeAnkleAngle(double theta)
{
  std::cout << "get ankle" << std::endl;
  int ankle_i = robot().jointIndexByName("RLEG_JOINT4");
  std::cout << "Get posture" << std::endl;
  auto p = hrp2postureTask->posture();
  std::cout << "Change posture" << std::endl;
  p[ankle_i][0] = theta;
  std::cout << "Set posture" << std::endl;
  hrp2postureTask->posture(p);
  std::cout << "Ankle angle " << theta << std::endl;
  return true;
}

}
