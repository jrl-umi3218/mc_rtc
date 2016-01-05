#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

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

  std::map<unsigned int, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;

  std::vector< std::map<int, std::vector<double> > > nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;

  std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;

public:
  std::vector<std::string> virtualLinks;
  std::vector<std::string> gripperLinks;
  std::map< std::string, std::vector<double> > halfSitting;
  std::map<int, sva::PTransformd> visual_tf;
  mc_rbdyn_urdf::Limits limits;
};

struct MC_ROBOTS_DLLAPI HRP2DRCRobotModule : public HRP2DRCCommonRobotModule
{
public:
  HRP2DRCRobotModule();

  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;

  virtual const std::vector< std::map<int, std::vector<double> > > & bounds() const;

  virtual const std::map< unsigned int, std::vector<double> > & stance() const;
public:
  std::vector<std::string> filteredLinks;
};

struct MC_ROBOTS_DLLAPI HRP2DRCGripperRobotModule : public HRP2DRCCommonRobotModule
{
public:
  HRP2DRCGripperRobotModule();

  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;

  virtual const std::vector< std::map<int, std::vector<double> > > & bounds() const;

  virtual const std::map< unsigned int, std::vector<double> > & stance() const;
public:
  std::vector<std::string> filteredLinks;
};

}

/*TODO Provide a different constructor to allow Gripper/NoGripper instantation */
ROBOT_MODULE_DEFAULT_CONSTRUCTOR("HRP2DRC", mc_robots::HRP2DRCGripperRobotModule)
