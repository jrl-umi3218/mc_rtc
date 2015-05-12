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
}

bool MCDrivingController::run()
{
  bool success = MCMRQPController::run();
  return success;
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
