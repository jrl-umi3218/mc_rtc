#include "mc_body6d_controller.h"

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

MCBody6dController::MCBody6dController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, double dt)
: MCController(robot_module, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addTask(postureTask.get());
  if(robot().name() == "hrp2_drc")
  {
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"), mc_rbdyn::Contact(robots(), "RFullSole", "AllGround")});
  }
  else if(robot().name() == "hrp4" || robot().name() == "jvrc-1")
  {
    solver().setContacts(
        {mc_rbdyn::Contact(robots(), "LeftFoot", "AllGround"), mc_rbdyn::Contact(robots(), "RightFoot", "AllGround")});
  }
  else
  {
    LOG_ERROR("MCBody6dController does not support robot " << robot().name())
    LOG_ERROR_AND_THROW(std::runtime_error, "MCBody6dController does not support your robot")
  }

  LOG_SUCCESS("MCBody6dController init done")
  if(robot().name() == "hrp2_drc")
  {
    efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(), 2.0, 1e5));
  }
  else if(robot().name() == "hrp4" || robot().name() == "jvrc-1")
  {
    efTask.reset(new mc_tasks::EndEffectorTask("r_wrist", robots(), robots().robotIndex(), 2.0, 1e5));
  }
  else
  {
    LOG_ERROR("MCBody6dController does not support robot " << robot().name())
    LOG_ERROR_AND_THROW(std::runtime_error, "MCBody6dController does not support your robot")
  }
  solver().addTask(efTask);
  comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  solver().addTask(comTask);
}

void MCBody6dController::reset(const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  efTask->reset();
  comTask->reset();
}

bool MCBody6dController::change_ef(const std::string & ef_name)
{
  if(robot().hasBody(ef_name))
  {
    solver().removeTask(efTask);
    postureTask->posture(robot().mbc().q);
    efTask.reset(new mc_tasks::EndEffectorTask(ef_name, robots(), robots().robotIndex()));
    solver().addTask(efTask);
    return true;
  }
  else
  {
    LOG_ERROR("Invalid link name: " << ef_name << ", control unchanged")
    return false;
  }
}

bool MCBody6dController::translate_ef(const Eigen::Vector3d & t)
{
  sva::PTransformd dtr(Eigen::Matrix3d::Identity(), t);
  efTask->add_ef_pose(dtr);
  return true;
}

bool MCBody6dController::rotate_ef(const Eigen::Matrix3d & m)
{
  sva::PTransformd dtr(m, Eigen::Vector3d(0, 0, 0));
  efTask->add_ef_pose(dtr);
  return true;
}

bool MCBody6dController::move_ef(const Eigen::Vector3d & t, const Eigen::Matrix3d & m)
{
  rotate_ef(m);
  translate_ef(t);
  return true;
}

} // namespace mc_control
