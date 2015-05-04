#include <mc_control/mc_drc_controller.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_drc_controller_services.cpp */

namespace mc_control
{

/* FIXME Seq loading is hardcoded for now... */
MCDRCGlobalController::MCDRCGlobalController()
: posture_controller(), body6d_controller(), com_controller(),
  seq_controller("/home/gergondet/catkin_ws/src/hrp2_drc/hrp2_drc_description/", "drc_stairs2", "/home/gergondet/devel-src/mcp/mc_rtc/mc_rtc_core/build_clang/drc_stairs_climbing.json"),
  //current_ctrl(POSTURE), next_ctrl(POSTURE),
  //controller(&posture_controller),
  //current_ctrl(BODY6D), next_ctrl(BODY6D),
  //controller(&body6d_controller),
  //current_ctrl(COM), next_ctrl(COM),
  //controller(&com_controller),
  current_ctrl(SEQ), next_ctrl(SEQ),
  controller(&seq_controller),
  next_controller(0)
{
}

void MCDRCGlobalController::init(const std::vector<double> & initq)
{
  std::vector<std::vector<double>> q;
  /*FIXME Get the position/attitude of the robot? */
  q.push_back({1,0,0,0,0,0,0.7593188263796187});

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
  setGripperCurrentQ(initq[23], initq[31]);
  controller->reset({q, {}});
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
    return r;
  }
  else
  {
    return false;
  }
}

const mc_control::QPResultMsg & MCDRCGlobalController::send(const double & t)
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

const mc_solver::QPSolver & MCDRCGlobalController::qpsolver() const
{
  return *(controller->qpsolver);
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

}
