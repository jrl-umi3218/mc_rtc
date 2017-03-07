#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

#include <mc_rbdyn/api.h>

namespace mc_robots
{

struct PrinterRobotModule: public mc_rbdyn::RobotModule
{
public:
  PrinterRobotModule();
protected:
  std::map<std::string, std::pair<std::string, std::string> > getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const;

  void readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks);

  std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;

  std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;

  std::vector< std::map<std::string, std::vector<double> > > nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;

  const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;

  const std::vector< std::map<std::string, std::vector<double> > >& bounds() const;

  const std::map<std::string, std::vector<double> > & stance() const;

public:

  std::vector<std::string> virtualLinks;
  std::map< std::string, std::vector<double> > halfSitting;
  mc_rbdyn_urdf::Limits limits;
  std::map<std::string, std::vector<sva::PTransformd> > visual_tfs;

};

}

ROBOT_MODULE_DEFAULT_CONSTRUCTOR("Printer", mc_robots::PrinterRobotModule);

