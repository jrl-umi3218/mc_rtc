#include <mc_control/generic_gripper.h>

#include <cmath>
#include <tinyxml2.h>

namespace mc_control
{

Mimic::Mimic()
: Mimic("", 0, 0)
{
}

Mimic::Mimic(const std::string & j, double m, double o)
: joint(j), multiplier(m), offset(o)
{
}

mimic_d_t readMimic(const std::string & urdf)
{
  mimic_d_t res;

  tinyxml2::XMLDocument doc;
  doc.Parse(urdf.c_str());
  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");

  if(robot)
  {
    std::vector<tinyxml2::XMLElement *> joints;
    // Extract all joint elements from the document
    {
      tinyxml2::XMLElement * joint = robot->FirstChildElement("joint");
      while(joint)
      {
        joints.push_back(joint);
        joint = joint->NextSiblingElement("joint");
      }
    }

    for(auto & j : joints)
    {
      std::string jointName = j->Attribute("name");
      tinyxml2::XMLElement * mimicDom = j->FirstChildElement("mimic");
      if(mimicDom)
      {
        std::string j = mimicDom->Attribute("joint");
        double m = 1;
        mimicDom->QueryDoubleAttribute("multiplier", &m);
        double o = 0;
        mimicDom->QueryDoubleAttribute("offset", &o);

        res[jointName] = Mimic(j, m, o);
      }
    }
  }

  return res;
}

unsigned int findSuccessorJoint(const mc_rbdyn::Robot & robot, unsigned int bodyIndex)
{
  for(int j = 0; j < robot.mb->nrJoints(); ++j)
  {
    if(robot.mb->predecessor(j) == static_cast<int>(bodyIndex))
    {
      return j;
    }
  }
  return robot.mb->nrJoints();
}

std::vector<std::string> gripperJoints(const mc_rbdyn::Robot & robot, const std::string & gripperName, const mimic_d_t & mimicDict)
{
  std::vector<std::string> res;

  unsigned int gripperBodyIndex = robot.bodyIndexByName(gripperName);
  unsigned int rootBodyIndex = robot.mb->parent(gripperBodyIndex);
  unsigned int rootJointIndex = findSuccessorJoint(robot, rootBodyIndex);
  std::string rootJointName = robot.mb->joint(rootJointIndex).name();

  res.push_back(rootJointName);
  for(const auto & m : mimicDict)
  {
    if(m.second.joint == rootJointName)
    {
      res.push_back(m.first);
    }
  }

  return res;
}

std::string findFirstCommonBody(const mc_rbdyn::Robot & robotFull, const std::string & bodyName, const mc_rbdyn::Robot & robot)
{
  if(robot.hasBody(bodyName))
  {
    return bodyName;
  }
  else
  {
    unsigned int bodyIndex = robotFull.bodyIndexByName(bodyName);
    int bodyPredIndex = robotFull.mb->parent(bodyIndex);
    if(bodyPredIndex == -1)
    {
      return "";
    }
    std::string bodyPredName = robotFull.mb->body(bodyPredIndex).name();
    return findFirstCommonBody(robotFull, bodyPredName, robot);
  }
}

Gripper::Gripper(const mc_rbdyn::Robot & robot, const std::string & gripperName,
                 const mc_rbdyn::Robot & controlRobot, const std::string & robot_urdf,
                 double currentQ, double timeStep)
: overCommandLimit(false), overCommandLimitIter(0), overCommandLimitIterN(10),
  actualQ(currentQ), actualCommandDiffTrigger(4*M_PI/180) /* 4 degress of difference */
{
  auto mimicDict = readMimic(robot_urdf);
  name = gripperJoints(robot, gripperName, mimicDict);
  unsigned int jointIndex = robot.jointIndexByName(name[0]);

  closeP = robot.ql[jointIndex][0];
  openP = robot.qu[jointIndex][0];
  vmax = std::min(std::abs(robot.vl[jointIndex][0]), robot.vu[jointIndex][0])/10;
  setCurrentQ(currentQ);
  this->timeStep = timeStep;

  mult.resize(name.size());
  _q.resize(name.size());
  for(size_t i = 0; i < name.size(); ++i)
  {
    if(i == 0)
    {
      mult[i] = 1;
      _q[i] = currentQ;
    }
    else
    {
      mult[i] = mimicDict[name[i]].multiplier;
      _q[i] = mult[i]*_q[0];
    }
  }

  unsigned int bodyIndex = robot.mb->predecessor(jointIndex);
  std::string bodyName = robot.mb->body(bodyIndex).name();
  controlBodyName = findFirstCommonBody(robot, bodyName, controlRobot);

  targetQIn = 0;
  targetQ = 0;
}

void Gripper::setCurrentQ(double currentQ)
{
  percentOpen = (currentQ - closeP)/(openP - closeP);
}

void Gripper::setTargetQ(double targetQ)
{
  targetQIn = targetQ;
  this->targetQ = &targetQIn;
}

void Gripper::setTargetOpening(double targetOpening)
{
  setTargetQ(curPosition() + (targetOpening - percentOpen)*(openP - closeP));
}

double Gripper::curPosition() const
{
  return closeP + (openP - closeP)*percentOpen;
}

void Gripper::setActualQ(double q)
{
  actualQ = q;
  double currentQ = curPosition();
  if(std::abs(actualQ - currentQ) > actualCommandDiffTrigger)
  {
    overCommandLimitIter++;
    if(overCommandLimitIter == overCommandLimitIterN)
    {
      std::cout << "Gripper safety triggered on " << name[0] << std::endl;
      overCommandLimit = true;
      setCurrentQ(actualQ - 0.01);
    }
  }
  else
  {
    overCommandLimitIter = 0;
    overCommandLimit = false;
  }
}

const std::vector<double> & Gripper::q()
{
  if(targetQ)
  {
    if(overCommandLimit || std::abs(curPosition() - targetQIn) < 0.001)
    {
      targetQ = 0;
    }
    else
    {
      if(targetQIn > curPosition())
      {
        percentOpen += std::min(vmax*timeStep, targetQIn - curPosition())/(openP - closeP);
      }
      else
      {
        percentOpen += std::max(-vmax*timeStep, targetQIn - curPosition())/(openP - closeP);
      }
    }
  }
  _q[0] = curPosition();
  for(size_t i = 1; i < _q.size(); ++i)
  {
    _q[i] = mult[i]*_q[0];
  }
  return _q;
}

}
