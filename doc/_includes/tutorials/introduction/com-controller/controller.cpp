#include "MyFirstController.h"

MyFirstController::MyFirstController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({
    {robots(), 0, 1, "LeftFoot", "AllGround"},
    {robots(), 0, 1, "RightFoot", "AllGround"}
  });
  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  solver().addTask(comTask);
  postureTask->stiffness(1);

  LOG_SUCCESS("MyFirstController init done " << this)
}

bool MyFirstController::run()
{
  if(std::abs(postureTask->posture()[jointIndex][0] - robot().mbc().q[jointIndex][0]) < 0.05)
  {
    switch_target();
  }
  if(comTask->eval().norm() < 0.01)
  {
    switch_com_target();
  }
  return mc_control::MCController::run();
}

void MyFirstController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
  comTask->reset();
  comZero = comTask->com();
}

void MyFirstController::switch_target()
{
  if(goingLeft)
  {
    postureTask->target({{"NECK_Y", robot().qu()[jointIndex]}});
  }
  else
  {
    postureTask->target({{"NECK_Y", robot().ql()[jointIndex]}});
  }
  goingLeft = !goingLeft;
}

void MyFirstController::switch_com_target()
{
  if(comDown)
  {
    comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2});
  }
  else
  {
    comTask->com(comZero);
  }
  comDown = !comDown;
}

CONTROLLER_CONSTRUCTOR("MyFirstController", MyFirstController)
