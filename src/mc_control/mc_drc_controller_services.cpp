#include <mc_control/mc_drc_controller.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_control
{

/* Specific to posture controller */
bool MCDRCPostureController::change_joint(int jid)
{
  if(jid < qpsolver->robots.robot().mb->nrJoints())
  {
    current_joint = jid + 1;
    return true;
  }
  else
  {
    std::cerr << "Invalid joint id given, control unchanged" << std::endl;
    return false;
  }
}
bool MCDRCPostureController::change_joint(const std::string & jname)
{
  if(qpsolver->robots.robot().hasJoint(jname))
  {
    current_joint = qpsolver->robots.robot().jointIndexByName(jname);
    return true;
  }
  else
  {
    std::cerr << "Invalid joint name: " << jname << ", control unchanged" << std::endl;
    return false;
  }
}

bool MCDRCPostureController::joint_up()
{
  add_joint_pos(0.01);
  return true;
}

bool MCDRCPostureController::joint_down()
{
  add_joint_pos(-0.01);
  return true;
}

void MCDRCPostureController::add_joint_pos(const double & v)
{
  auto p = postureTask->posture();
  p[current_joint][0] += v;
  postureTask->posture(p);
}

bool MCDRCPostureController::set_joint_pos(const std::string & jname, const double & v)
{
  if(qpsolver->robots.robot().hasJoint(jname))
  {
    unsigned int jid = qpsolver->robots.robot().jointIndexByName(jname);
    auto p = postureTask->posture();
    p[jid][0] = v;
    postureTask->posture(p);
    return true;
  }
  else
  {
    std::cerr << "Invalid joint name " << jname << " provided" << std::endl;
    return false;
  }
}

bool MCDRCBody6dController::change_ef(const std::string & ef_name)
{
  if(robot().hasBody(ef_name))
  {
    efTask->removeFromSolver(*qpsolver);
    postureTask->posture(robot().mbc->q);
    efTask.reset(new EndEffectorTask(ef_name, qpsolver->robots, qpsolver->robots.robotIndex));
    efTask->addToSolver(*qpsolver);
    return true;
  }
  else
  {
    std::cerr << "Invalid link name: " << ef_name << ", control unchanged" << std::endl;
    return false;
  }
}

bool MCDRCBody6dController::translate_ef(const Eigen::Vector3d & t)
{
  sva::PTransformd dtr(Eigen::Matrix3d::Identity(), t);
  efTask->add_ef_pose(dtr);
  return true;
}

bool MCDRCBody6dController::rotate_ef(const Eigen::Matrix3d & m)
{
  sva::PTransformd dtr(m, Eigen::Vector3d(0,0,0));
  efTask->add_ef_pose(dtr);
  return true;
}

/* Called by the RT component to access PostureController service */
bool MCDRCGlobalController::change_joint(int jid)
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.change_joint(jid);
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::change_joint(const std::string & jname)
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.change_joint(jname);
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::joint_up()
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.joint_up();
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::joint_down()
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.joint_down();
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::set_joint_pos(const std::string & jname, const double & pos)
{
  if(current_ctrl == POSTURE)
  {
    return posture_controller.set_joint_pos(jname, pos);
  }
  else
  {
    return false;
  }
}

/* Called by the RT component to access Body6dController service */
bool MCDRCGlobalController::change_ef(const std::string & ef_name)
{
  if(current_ctrl == BODY6D)
  {
    return body6d_controller.change_ef(ef_name);
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::translate_ef(const Eigen::Vector3d & t)
{
  if(current_ctrl == BODY6D)
  {
    return body6d_controller.translate_ef(t);
  }
  else
  {
    return false;
  }
}
bool MCDRCGlobalController::rotate_ef(const Eigen::Matrix3d & m)
{
  if(current_ctrl == BODY6D)
  {
    return body6d_controller.rotate_ef(m);
  }
  else
  {
    return false;
  }
}

}
