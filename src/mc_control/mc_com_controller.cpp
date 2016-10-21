#include "mc_com_controller.h"
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_rbdyn/Surface.h>

#include <mc_rtc/logging.h>

namespace mc_control
{

MCCoMController::MCCoMController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->addTask(postureTask.get());
  if(robot().name() == "hrp2_drc")
  {
    qpsolver->setContacts({
      mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
      mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")
    });
  }
  else if(robot().name() == "hrp4")
  {
    qpsolver->setContacts({
      mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"),
      mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")
    });
  }
  else
  {
    LOG_ERROR("MCBody6dController does not support robot " << robot().name())
    throw("MCBody6dController does not support your robot");
  }

  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  solver().addTask(comTask);
}

void MCCoMController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  comTask->reset();
}

bool MCCoMController::move_com(const Eigen::Vector3d & v)
{
  comTask->move_com(v);
  return true;
}

}
