#include <mc_control/mc_drc_controller.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_robots/polaris_ranger.h>
#include <mc_robots/polaris_ranger_egress.h>

/* Note all service calls except for controller switches are implemented in mc_drc_controller_services.cpp */

namespace mc_control
{

/* FIXME Seq loading is hardcoded for now... */
MCDRCGlobalController::MCDRCGlobalController()
: posture_controller(), body6d_controller(), com_controller(),
  seq_controller("/home/hrp2user/jrl/hrp2_drc/hrp2_drc_description/", "drc_stairs2", "/home/hrp2user/jrl/mc_rtc/data/drc_stairs_climbing.json"),
  driving_controller({std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::PolarisRangerRobotModule()),
                      std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::EnvRobotModule("/home/hrp2user/jrl/mc_env_description/", "ground"))}),
  egress_controller("/home/hrp2user/jrl/hrp2_drc/hrp2_drc_description/", "polaris_ranger"),
  egress_mrqp_controller({std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::PolarisRangerEgressRobotModule()),
                      std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::EnvRobotModule("/home/hrp2user/jrl/mc_env_description/", "ground"))}),
  //current_ctrl(POSTURE), next_ctrl(POSTURE),
  //controller(&posture_controller),
  //current_ctrl(BODY6D), next_ctrl(BODY6D),
  //controller(&body6d_controller),
  //current_ctrl(COM), next_ctrl(COM),
  //controller(&com_controller),
  //current_ctrl(SEQ), next_ctrl(SEQ),
  //controller(&seq_controller),
  //current_ctrl(DRIVING), next_ctrl(DRIVING),
  //controller(&driving_controller),
  //current_ctrl(EGRESS), next_ctrl(EGRESS),
  //controller(&egress_controller),
  current_ctrl(EGRESS_MRQP), next_ctrl(EGRESS_MRQP),
  controller(&egress_mrqp_controller),
  next_controller(0)
{
}

void MCDRCGlobalController::init(const std::vector<double> & initq)
{
  std::vector<std::vector<double>> q;
  /*FIXME Get the position/attitude of the robot? */
  //q.push_back({1,0,0,0,-0.275,-0.05,0.76});
  if(current_ctrl == SEQ)
  {
    q.push_back({0.999735735669,-0.000712226669955,0.0227020230143,-0.00354537868489,-0.364642021503,-0.00254821664029,0.762505884771});
  }
  else if(current_ctrl == EGRESS)
  {
    q.push_back({1, 0, 0, 0, 0, 0, 0.76});
  }
  else
  {
    q.push_back({1, 0, 0, 0, 0, 0, 0.76});
  }

  /* The OpenRTM components don't give q in the same order as the QP */
  for(size_t i = 0; i < 24; ++i) // until RARM_LINK7
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 32; i < 37; ++i) // RHAND
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 24; i < 32; ++i) // LARM_LINK*
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 37; i < 42; ++i) // LHAND
  {
    q.push_back({initq[i]});
  }
  setGripperCurrentQ(initq[31], initq[23]);
  controller->reset({q, {}});
}

void MCDRCGlobalController::setSensorOrientation(const Eigen::Vector3d & rpy)
{
  controller->sensorOri = rpy;
}

void MCDRCGlobalController::setEncoderValues(const std::vector<double> & eValues)
{
  controller->encoderValues = eValues;
}

void MCDRCGlobalController::setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches)
{
  controller->setWrenches(wrenches);
}

void MCDRCGlobalController::setActualGripperQ(double rQ, double lQ)
{
  controller->rgripper->setActualQ(rQ);
  controller->lgripper->setActualQ(lQ);
}

bool MCDRCGlobalController::run()
{
  /* Check if we need to change the controller this time */
  if(next_controller)
  {
    std::cout << "Switching controllers" << std::endl;
    if(!running)
    {
      controller = next_controller;
    }
    else
    {
      /*XXX Need to be careful here */
      /*XXX Need to get the current contacts from the controller when needed */
      std::cout << "Reset with q[0]" << std::endl;
      std::cout << controller->robot().mbc->q[0][0] << " ";
      std::cout << controller->robot().mbc->q[0][1] << " ";
      std::cout << controller->robot().mbc->q[0][2] << " ";
      std::cout << controller->robot().mbc->q[0][3] << " ";
      std::cout << controller->robot().mbc->q[0][4] << " ";
      std::cout << controller->robot().mbc->q[0][5] << " ";
      std::cout << controller->robot().mbc->q[0][6] << std::endl;
      next_controller->lgripper->setCurrentQ(controller->lgripper->curPosition());
      next_controller->rgripper->setCurrentQ(controller->rgripper->curPosition());
      next_controller->reset({controller->robot().mbc->q, {}});
      controller = next_controller;
    }
    next_controller = 0;
    current_ctrl = next_ctrl;
  }
  if(running)
  {
    bool r = controller->run();
    if(!r) { running = false; }
    return r;
  }
  else
  {
    return false;
  }
}

const QPResultMsg & MCDRCGlobalController::send(const double & t)
{
  return controller->send(t);
}

const std::vector<double> & MCDRCGlobalController::gripperQ(bool lgripper)
{
  if(lgripper)
  {
    return controller->lgripper->q();
  }
  else
  {
    return controller->rgripper->q();
  }
}

void MCDRCGlobalController::setGripperCurrentQ(double lQ, double rQ)
{
  controller->lgripper->setCurrentQ(lQ);
  controller->rgripper->setCurrentQ(rQ);
}

void MCDRCGlobalController::setGripperTargetQ(double lQ, double rQ)
{
  controller->lgripper->setTargetQ(lQ);
  controller->rgripper->setTargetQ(rQ);
}

void MCDRCGlobalController::setLGripperTargetQ(double lQ)
{
  controller->lgripper->setTargetQ(lQ);
}

void MCDRCGlobalController::setRGripperTargetQ(double rQ)
{
  controller->rgripper->setTargetQ(rQ);
}

void MCDRCGlobalController::setGripperOpenPercent(double lQ, double rQ)
{
  controller->lgripper->setTargetOpening(lQ);
  controller->rgripper->setTargetOpening(rQ);
}

bool MCDRCGlobalController::EnablePostureController()
{
  next_ctrl = POSTURE;
  if(current_ctrl != POSTURE)
  {
    next_controller = &posture_controller;
  }
  //while(next_controller != 0);
  return true;
}

bool MCDRCGlobalController::EnableBody6dController()
{
  next_ctrl = BODY6D;
  if(current_ctrl != BODY6D)
  {
    next_controller = &body6d_controller;
  }
  //while(next_controller != 0);
  return true;
}

bool MCDRCGlobalController::EnableCoMController()
{
  next_ctrl = COM;
  if(current_ctrl != COM)
  {
    next_controller = &com_controller;
  }
  return true;
}

bool MCDRCGlobalController::EnableSeqController()
{
  next_ctrl = SEQ;
  if(current_ctrl != SEQ)
  {
    next_controller = &seq_controller;
  }
  return true;
}

bool MCDRCGlobalController::EnableDrivingController()
{
  next_ctrl = DRIVING;
  if(current_ctrl != DRIVING)
  {
    next_controller = &driving_controller;
  }
  return true;
  return false;
}

}
