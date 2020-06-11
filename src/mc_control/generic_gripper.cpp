/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/generic_gripper.h>

#include <mc_filter/utils/clamp.h>

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/constants.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <tinyxml2.h>

namespace mc_control
{

namespace
{

/* Returns the mimic joints stored in the URDF model */
std::vector<mc_rbdyn::Mimic> readMimic(const std::string & urdf)
{
  std::vector<mc_rbdyn::Mimic> res;

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

        res.push_back({jointName, j, m, o});
      }
    }
  }

  return res;
}

/* Returns all joints associated to a gripper's active joints */
std::vector<mc_rbdyn::Mimic> gripperMimics(const std::vector<std::string> & jointNames,
                                           const std::vector<mc_rbdyn::Mimic> & mimics)
{
  std::vector<mc_rbdyn::Mimic> res;

  for(const auto & gripperName : jointNames)
  {
    for(const auto & m : mimics)
    {
      if(m.joint == gripperName)
      {
        res.push_back(m);
      }
    }
  }

  return res;
}

} // namespace

Gripper::Gripper(const mc_rbdyn::Robot & robot,
                 const std::vector<std::string> & jointNames,
                 const std::string & robot_urdf,
                 bool reverseLimits,
                 const mc_rbdyn::RobotModule::Gripper::Safety & safety)
: Gripper(robot, jointNames, gripperMimics(jointNames, readMimic(robot_urdf)), reverseLimits, safety)
{
}

Gripper::Gripper(const mc_rbdyn::Robot & robot,
                 const std::vector<std::string> & jointNames,
                 const std::vector<mc_rbdyn::Mimic> & mimics,
                 bool reverseLimits,
                 const mc_rbdyn::RobotModule::Gripper::Safety & safety)
: actualQ(jointNames.size(), 0), config_(safety), savedConfig_(safety), defaultConfig_(safety)
{
  active_joints = jointNames;
  mult.resize(0);
  _q.resize(0);
  auto getReferenceIdx = [&](const std::string & joint) {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      const auto & rji = rjo[i];
      if(rji == joint)
      {
        return i;
      }
    }
    mc_rtc::log::error_and_throw<std::runtime_error>("Active joint {} for {} is not part of the reference joint order",
                                                     joint, robot.name());
  };
  for(size_t i = 0; i < jointNames.size(); ++i)
  {
    const auto & name = jointNames[i];
    if(robot.hasJoint(name))
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
      vmax.push_back(std::min(std::abs(robot.vl()[jointIndex][0]), robot.vu()[jointIndex][0]));
      _q.push_back(actualQ[i]);
    }
    else
    {
      mc_rtc::log::error("Gripper active joint {} is not part of the loaded robot, limits are unknown", name);
      closeP.push_back(-0.01);
      openP.push_back(0.01);
      vmax.push_back(0);
      _q.push_back(0);
    }
    active_joints_idx.push_back(getReferenceIdx(name));
    mult.push_back({i, 1.0});
    offset.push_back(0.0);
  }
  names = jointNames;
  auto getActiveIdx = [this](const std::string & joint) {
    for(size_t i = 0; i < active_joints.size(); ++i)
    {
      if(active_joints[i] == joint)
      {
        return i;
      }
    }
    mc_rtc::log::error_and_throw<std::runtime_error>("Trying to mimic non existant joint: {}", joint);
  };
  for(const auto & m : mimics)
  {
    names.push_back(m.name);
    auto jIndex = getActiveIdx(m.joint);
    _q.push_back(m.multiplier * _q[jIndex] + m.offset);
    mult.push_back({jIndex, m.multiplier});
    offset.push_back(m.offset);
  }

  percentOpen.resize(actualQ.size());
  overCommandLimit.resize(actualQ.size());
  overCommandLimitIter.resize(actualQ.size());

  targetQIn = {};
  targetQ = nullptr;

  reversed = reverseLimits;

  joints_mbc_idx.clear();
  for(const auto & name : names)
  {
    if(robot.hasJoint(name))
    {
      joints_mbc_idx.push_back(robot.jointIndexByName(name));
    }
    else
    {
      joints_mbc_idx.push_back(-1);
    }
  }
}

void Gripper::resetDefaults()
{
  config_ = defaultConfig_;
}

void Gripper::saveConfig()
{
  savedConfig_ = config_;
}

void Gripper::restoreConfig()
{
  config_ = savedConfig_;
}

void Gripper::reset(const std::vector<double> & currentQ)
{
  for(size_t i = 0; i < percentOpen.size(); ++i)
  {
    percentOpen[i] = (currentQ[active_joints_idx[i]] - closeP[i]) / (openP[i] - closeP[i]);
  }
}

void Gripper::reset(const Gripper & gripper)
{
  this->percentOpen = gripper.percentOpen;
}

void Gripper::setTargetQ(const std::vector<double> & targetQ)
{
  targetQIn = targetQ;
  this->targetQ = &targetQIn;
}

void Gripper::setTargetOpening(double targetOpening)
{
  targetOpening = mc_filter::utils::clamp(targetOpening, 0, 1);
  auto cur = curPosition();
  std::vector<double> targetQin(cur.size());
  for(size_t i = 0; i < targetQin.size(); ++i)
  {
    targetQin[i] = cur[i] + (targetOpening - percentOpen[i]) * (openP[i] - closeP[i]);
  }
  setTargetQ(targetQin);
}

void Gripper::percentVMAX(double percent)
{
  config_.percentVMax = mc_filter::utils::clamp(percent, 0, 1);
}

double Gripper::percentVMAX() const
{
  return config_.percentVMax;
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

void Gripper::run(double timeStep, mc_rbdyn::Robot & robot, std::map<std::string, std::vector<double>> & qOut)
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
          percentOpen[i] +=
              std::min(vmax[i] * config_.percentVMax * timeStep, targetQIn[i] - cur[i]) / (openP[i] - closeP[i]);
        }
        else
        {
          percentOpen[i] +=
              std::max(-vmax[i] * config_.percentVMax * timeStep, targetQIn[i] - cur[i]) / (openP[i] - closeP[i]);
        }
      }
      reached = reached && i_reached;
    }
    if(reached)
    {
      targetQ = nullptr;
    }
  }
  const auto & q = robot.encoderValues();
  auto currentQ = curPosition();
  if(q.size())
  {
    for(size_t i = 0; i < active_joints_idx.size(); ++i)
    {
      actualQ[i] = q[active_joints_idx[i]];
    }
  }
  else
  {
    actualQ = currentQ;
  }
  for(size_t i = 0; i < active_joints.size(); ++i)
  {
    _q[i] = currentQ[i];
    if(joints_mbc_idx[i] != -1)
    {
      robot.mbc().q[joints_mbc_idx[i]] = {_q[i]};
    }
    qOut[names[i]] = {_q[i]};
  }
  for(size_t i = active_joints.size(); i < names.size(); ++i)
  {
    _q[i] = mult[i].second * _q[mult[i].first] + offset[i];
    if(joints_mbc_idx[i] != -1)
    {
      robot.mbc().q[joints_mbc_idx[i]] = {_q[i]};
    }
    qOut[names[i]] = {_q[i]};
  }
  for(size_t i = 0; i < actualQ.size(); ++i)
  {
    if(std::abs(actualQ[i] - currentQ[i]) > config_.actualCommandDiffTrigger)
    {
      overCommandLimitIter[i]++;
      if(overCommandLimitIter[i] == config_.overCommandLimitIterN)
      {
        mc_rtc::log::warning("Gripper safety triggered on {}", names[i]);
        overCommandLimit[i] = true;
        if(reversed)
        {
          actualQ[i] = actualQ[i] + config_.releaseSafetyOffset;
        }
        else
        {
          actualQ[i] = actualQ[i] - config_.releaseSafetyOffset;
        }
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

double Gripper::opening() const
{
  return std::accumulate(percentOpen.begin(), percentOpen.end(), 0.0) / static_cast<double>(percentOpen.size());
}

bool Gripper::complete() const
{
  return targetQ == nullptr;
}

} // namespace mc_control
