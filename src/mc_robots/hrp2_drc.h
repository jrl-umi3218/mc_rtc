#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

#include <mc_rtc/logging.h>

#include "api.h"

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI HRP2DRCCommonRobotModule : public mc_rbdyn::RobotModule
{
public:
  HRP2DRCCommonRobotModule();
protected:
  std::map<std::string, std::pair<std::string, std::string> > getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const;

  void readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks);

  std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;

  std::vector< std::map<std::string, std::vector<double> > > nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;

  std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;

public:
  std::vector<std::string> virtualLinks;
  std::vector<std::string> gripperLinks;
  std::map< std::string, std::vector<double> > halfSitting;
  mc_rbdyn_urdf::Limits limits;
};

struct MC_ROBOTS_DLLAPI HRP2DRCRobotModule : public HRP2DRCCommonRobotModule
{
public:
  HRP2DRCRobotModule();

  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;

  virtual const std::vector< std::map<std::string, std::vector<double> > > & bounds() const;

  virtual const std::map<std::string, std::vector<double> > & stance() const;
public:
  std::vector<std::string> filteredLinks;
};

struct MC_ROBOTS_DLLAPI HRP2DRCGripperRobotModule : public HRP2DRCCommonRobotModule
{
public:
  HRP2DRCGripperRobotModule();

  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;

  virtual const std::vector< std::map<std::string, std::vector<double> > > & bounds() const;

  virtual const std::map<std::string, std::vector<double> > & stance() const;
public:
  std::vector<std::string> filteredLinks;
};

}

extern "C"
{
  ROBOT_MODULE_API std::vector<std::string> MC_RTC_ROBOT_MODULE()
  {
    return {"HRP2DRC", "HRP2DRCNoGripper"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr) { delete ptr; }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & n)
  {
    if(n == "HRP2DRC")
    {
      return new mc_robots::HRP2DRCGripperRobotModule();
    }
    else if(n == "HRP2DRCNoGripper")
    {
      return new mc_robots::HRP2DRCRobotModule();
    }
    else
    {
      LOG_ERROR("HRP2DRC module Cannot create an object of type " << n)
      return nullptr;
    }
  }
}
