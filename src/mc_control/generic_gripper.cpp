#include <mc_control/generic_gripper.h>
#include <mc_rtc/logging.h>

#include <cmath>
#include <numeric>
#include <tinyxml2.h>

namespace mc_control
{

namespace
{

/* Returns the mimic joints stored in the URDF model */
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

        res[jointName] = {j, m, o};
      }
    }
  }

  return res;
}

/* Returns all joints associated to a gripper's active joints */
std::vector<std::string> gripperJoints(const std::vector<std::string> & jointNames, const mimic_d_t & mimicDict)
{
  std::vector<std::string> res;

  for(const auto & gripperName : jointNames)
  {
    res.push_back(gripperName);
    for(const auto & m : mimicDict)
    {
      if(m.second.joint == gripperName)
      {
        res.push_back(m.first);
      }
    }
  }

  return res;
}

} // namespace

Gripper::Gripper(const mc_rbdyn::Robot & robot,
                 const std::vector<std::string> & jointNames,
                 const std::string & robot_urdf,
                 const std::vector<double> & currentQ,
                 double timeStep,
                 bool reverseLimits)
: overCommandLimitIter(0), overCommandLimitIterN(5), actualQ(currentQ),
  actualCommandDiffTrigger(8 * M_PI / 180) /* 8 degress of difference */
{
  auto mimicDict = readMimic(robot_urdf);
  names = gripperJoints(jointNames, mimicDict);
  active_joints = jointNames;
  active_idx.resize(0);
  mult.resize(0);
  _q.resize(0);
  unsigned int j = 0;
  for(size_t i = 0; i < names.size(); ++i)
  {
    const auto & name = names[i];
    if(robot.hasJoint(name) && mimicDict.count(name) == 0)
    {
      unsigned int jointIndex = robot.jointIndexByName(name);
      if(!reverseLimits)
      {
        closeP.push_back(robot.ql()[jointIndex][0]);
        openP.push_back(robot.qu()[jointIndex][0]);
      }
      else
      {
        closeP.push_back(robot.qu()[jointIndex][0]);
        openP.push_back(robot.ql()[jointIndex][0]);
      }
      vmax.push_back(std::min(std::abs(robot.vl()[jointIndex][0]), robot.vu()[jointIndex][0]) / 4);
      active_idx.push_back(i);
      _q.push_back(currentQ[j]);
      ++j;
    }
    else
    {
      _q.push_back(0);
    }
  }
  for(size_t i = 0; i < _q.size(); ++i)
  {
    auto it = std::find(active_idx.begin(), active_idx.end(), i);
    if(it == active_idx.end())
    {
      for(size_t j = 0; j < active_joints.size(); ++j)
      {
        if(active_joints[j] == mimicDict.at(names[i]).joint)
        {
          mult.push_back({j, mimicDict[names[i]].multiplier});
          break;
        }
      }
    }
    else
    {
      mult.push_back({i, 1.0});
    }
  }
  for(size_t i = 0; i < mult.size(); ++i)
  {
    _q[i] = _q[mult[i].first] * mult[i].second;
  }

  percentOpen.resize(currentQ.size());
  overCommandLimit.resize(currentQ.size());
  overCommandLimitIter.resize(currentQ.size());
  setCurrentQ(currentQ);
  this->timeStep = timeStep;

  targetQIn = {};
  targetQ = 0;
}

void Gripper::setCurrentQ(const std::vector<double> & currentQ)
{
  for(size_t i = 0; i < percentOpen.size(); ++i)
  {
    percentOpen[i] = (currentQ[i] - closeP[i]) / (openP[i] - closeP[i]);
  }
}

void Gripper::setTargetQ(const std::vector<double> & targetQ)
{
  targetQIn = targetQ;
  this->targetQ = &targetQIn;
}

void Gripper::setTargetOpening(double targetOpening)
{
  auto cur = curPosition();
  std::vector<double> targetQin(cur.size());
  for(size_t i = 0; i < targetQin.size(); ++i)
  {
    targetQin[i] = cur[i] + (targetOpening - percentOpen[i]) * (openP[i] - closeP[i]);
  }
  setTargetQ(targetQin);
}

std::vector<double> Gripper::curPosition() const
{
  std::vector<double> res(percentOpen.size());
  for(size_t i = 0; i < res.size(); ++i)
  {
    res[i] = closeP[i] + (openP[i] - closeP[i]) * percentOpen[i];
  }
  return res;
}

void Gripper::setActualQ(const std::vector<double> & q)
{
  actualQ = q;
  auto currentQ = curPosition();
  for(size_t i = 0; i < actualQ.size(); ++i)
  {
    if(std::abs(actualQ[i] - currentQ[i]) > actualCommandDiffTrigger)
    {
      overCommandLimitIter[i]++;
      if(overCommandLimitIter[i] == overCommandLimitIterN)
      {
        LOG_WARNING("Gripper safety triggered on " << names[active_idx[i]])
        overCommandLimit[i] = true;
        actualQ[i] = actualQ[i] - 2 * M_PI / 180;
        setTargetQ(actualQ);
      }
    }
    else
    {
      overCommandLimitIter[i] = 0;
      overCommandLimit[i] = false;
    }
  }
}

const std::vector<double> & Gripper::q()
{
  if(targetQ)
  {
    auto cur = curPosition();
    bool reached = true;
    for(size_t i = 0; i < cur.size(); ++i)
    {
      bool i_reached = std::abs(cur[i] - targetQIn[i]) < 0.001;
      if(!i_reached)
      {
        if(targetQIn[i] > cur[i])
        {
          percentOpen[i] += std::min(vmax[i] * timeStep, targetQIn[i] - cur[i]) / (openP[i] - closeP[i]);
        }
        else
        {
          percentOpen[i] += std::max(-vmax[i] * timeStep, targetQIn[i] - cur[i]) / (openP[i] - closeP[i]);
        }
      }
      reached = reached && i_reached;
    }
    if(reached)
    {
      targetQ = 0;
    }
  }
  auto cur = curPosition();
  for(size_t i = 0; i < active_idx.size(); ++i)
  {
    _q[active_idx[i]] = cur[i];
  }
  for(size_t i = 0; i < _q.size(); ++i)
  {
    _q[i] = mult[i].second * _q[mult[i].first];
  }
  return _q;
}

double Gripper::opening() const
{
  return std::accumulate(percentOpen.begin(), percentOpen.end(), 0.0) / percentOpen.size();
}

} // namespace mc_control
