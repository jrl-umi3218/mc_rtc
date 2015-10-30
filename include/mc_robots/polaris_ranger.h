#ifndef _H_MCROBOTSPOLARISRANGER_H_
#define _H_MCROBOTSPOLARISRANGER_H_

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn_urdf/urdf.h>

namespace mc_robots
{

struct PolarisRangerRobotModule : public mc_rbdyn::RobotModule
{
public:
  PolarisRangerRobotModule(bool is_interactive = true);
protected:
  std::map<std::string, std::pair<std::string, std::string> > getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const;

  void readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks);

  std::map<unsigned int, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;

  std::vector< std::map<int, std::vector<double> > > nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;


  std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;

  const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const;

  const std::vector< std::map<int, std::vector<double> > >& bounds() const;

  const std::map< unsigned int, std::vector<double> > & stance() const;
public:
  std::vector<std::string> virtualLinks;
  std::vector<std::string> gripperLinks;
  std::map< std::string, std::vector<double> > halfSitting;
  std::map<int, sva::PTransformd> visual_tf;
  mc_rbdyn_urdf::Limits limits;
};

}
#endif
