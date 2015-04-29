#include <mc_control/mc_posture_controller.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

/* Note all service calls except for controller switches are implemented in mc_drc_controller_services.cpp */

namespace mc_control
{

/* Common stuff */
MCPostureController::MCPostureController()
: MCController(), current_joint(1)
{
  qpsolver->setContacts({});

  qpsolver->addConstraintSet(contactConstraint);
  qpsolver->addConstraintSet(dynamicsConstraint);
  qpsolver->addConstraintSet(selfCollisionConstraint);
  qpsolver->setContacts({
    mc_rbdyn::Contact(robot().surfaces.at("LFullSole"), env().surfaces.at("AllGround")),
    mc_rbdyn::Contact(robot().surfaces.at("RFullSole"), env().surfaces.at("AllGround"))
  });
  qpsolver->solver.addTask(postureTask.get());

  std::cout << "MCPostureController init done " << this << std::endl;
}

/* Specific to posture controller */
bool MCPostureController::change_joint(int jid)
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
bool MCPostureController::change_joint(const std::string & jname)
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

bool MCPostureController::joint_up()
{
  add_joint_pos(0.01);
  return true;
}

bool MCPostureController::joint_down()
{
  add_joint_pos(-0.01);
  return true;
}

void MCPostureController::add_joint_pos(const double & v)
{
  auto p = postureTask->posture();
  p[current_joint][0] += v;
  postureTask->posture(p);
}

bool MCPostureController::set_joint_pos(const std::string & jname, const double & v)
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

}
