#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/logging.h>

#include <mc_rbdyn_urdf/urdf.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI HRP4CommonRobotModule : public mc_rbdyn::RobotModule
{
public:
  HRP4CommonRobotModule();

protected:
  std::map<std::string, std::pair<std::string, std::string>> getConvexHull(
      const std::map<std::string, std::pair<std::string, std::string>> & files) const;

  void readUrdf(const std::string & robotName, bool fixed, const std::vector<std::string> & filteredLinks);

  std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;

  std::vector<std::map<std::string, std::vector<double>>> nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;

  std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;

  void init();

public:
  std::vector<std::string> virtualLinks;
  std::vector<std::string> gripperLinks;
  std::map<std::string, std::vector<double>> halfSitting;
  mc_rbdyn_urdf::Limits limits;
};

struct MC_ROBOTS_DLLAPI HRP4NoHandRobotModule : public HRP4CommonRobotModule
{
public:
  HRP4NoHandRobotModule(bool fixed);
};

struct MC_ROBOTS_DLLAPI HRP4WithHandRobotModule : public HRP4CommonRobotModule
{
public:
  HRP4WithHandRobotModule(bool fixed);
};

struct MC_ROBOTS_DLLAPI HRP4VREPRobotModule : public HRP4WithHandRobotModule
{
public:
  HRP4VREPRobotModule(bool fixed);
};

struct MC_ROBOTS_DLLAPI HRP4FlexRobotModule : public HRP4WithHandRobotModule
{
public:
  HRP4FlexRobotModule(bool fixed);
};

struct MC_ROBOTS_DLLAPI HRP4ComanoidRobotModule : public HRP4CommonRobotModule
{
public:
  HRP4ComanoidRobotModule();
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"HRP4",          "HRP4NoHand",    "HRP4VREP",    "HRP4Flex", "HRP4Fixed", "HRP4NoHandFixed",
             "HRP4VREPFixed", "HRP4FlexFixed", "HRP4Comanoid"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    if(n == "HRP4")
    {
      return new mc_robots::HRP4WithHandRobotModule(false);
    }
    else if(n == "HRP4NoHand")
    {
      return new mc_robots::HRP4NoHandRobotModule(false);
    }
    else if(n == "HRP4VREP")
    {
      return new mc_robots::HRP4VREPRobotModule(false);
    }
    else if(n == "HRP4Fixed")
    {
      return new mc_robots::HRP4WithHandRobotModule(true);
    }
    else if(n == "HRP4NoHandFixed")
    {
      return new mc_robots::HRP4NoHandRobotModule(true);
    }
    else if(n == "HRP4VREPFixed")
    {
      return new mc_robots::HRP4VREPRobotModule(true);
    }
    else if(n == "HRP4Flex")
    {
      return new mc_robots::HRP4FlexRobotModule(false);
    }
    else if(n == "HRP4FlexFixed")
    {
      return new mc_robots::HRP4FlexRobotModule(true);
    }
    else if(n == "HRP4Comanoid")
    {
      return new mc_robots::HRP4ComanoidRobotModule();
    }
    else
    {
      LOG_ERROR("HRP4 module Cannot create an object of type " << n)
      return nullptr;
    }
  }
}
