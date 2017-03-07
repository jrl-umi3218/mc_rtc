#include "printer.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <fstream>

namespace mc_robots
{

PrinterRobotModule::PrinterRobotModule()
: RobotModule(mc_rtc::MC_ENV_DESCRIPTION_PATH, "printer")
{
  halfSitting["printer"] = {};
  halfSitting["rack"] = {-0.02};

  _flexibility.push_back(mc_rbdyn::Flexibility{"rack", 0., 2., 0.});

  readUrdf("printer", virtualLinks);
}

std::map<std::string, std::pair<std::string, std::string> > PrinterRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const
{
  std::string convexPath = path + "/convex/printer/";
  std::map<std::string, std::pair<std::string, std::string> > res;
  for(const auto & f : files)
  {
    res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
  }
  return res;
}

std::map<std::string, std::pair<std::string, std::string>> PrinterRobotModule::stdCollisionsFiles(const rbd::MultiBody &/*mb*/) const
{
  std::map<std::string, std::pair<std::string, std::string>> res;

  return res;
}


void PrinterRobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
{
  urdf_path = path + "/urdf/" + robotName + ".urdf";
  std::ifstream ifs(urdf_path);
  if(ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), true, filteredLinks);
    mb = res.mb;
    mbc = res.mbc;
    mbg = res.mbg;
    limits = res.limits;

    std::vector<sva::PTransformd> tfs;
    for(const auto pair : res.visual)
    {
      tfs.clear();
      for(const auto visual : pair.second)
      {
        tfs.push_back(visual.origin);
      }
      visual_tfs[pair.first] = tfs;
    }

    _collisionTransforms = res.collision_tf;
  }
  else
  {
    LOG_ERROR("Could not open printer model at " << urdf_path)
    throw("Failed to open printer model");
  }
}

std::map<std::string, std::vector<double>> PrinterRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
{
  std::map<std::string, std::vector<double>> res;
  for(const auto & j : mb.joints())
  {
    if(j.name() != "Root")
    {
      res[j.name()] = halfSitting.at(j.name());
      for(auto & ji : res[j.name()])
      {
        ji = M_PI*ji/180;
      }
    }
  }
  return res;
}

std::vector< std::map<std::string, std::vector<double> > > PrinterRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits & limits) const
{
  std::vector< std::map<std::string, std::vector<double> > > res(0);
  res.push_back(limits.lower);
  res.push_back(limits.upper);
  {
    auto mvelocity = limits.velocity;
    for(auto & mv : mvelocity)
    {
      for(auto & mvi : mv.second)
      {
        mvi = -mvi;
      }
    }
    res.push_back(mvelocity);
  }
  res.push_back(limits.velocity);
  {
    auto mtorque = limits.torque;
    for(auto & mt : mtorque)
    {
      for(auto & mti : mt.second)
      {
        mti = -mti;
      }
    }
    res.push_back(mtorque);
  }
  res.push_back(limits.torque);
  return res;
}

const std::map<std::string, std::pair<std::string, std::string> > & PrinterRobotModule::convexHull() const
{
  auto fileByBodyName = stdCollisionsFiles(mb);
  const_cast<PrinterRobotModule*>(this)->_convexHull = getConvexHull(fileByBodyName);
  return _convexHull;
}

const std::vector< std::map<std::string, std::vector<double> > > & PrinterRobotModule::bounds() const
{
  const_cast<PrinterRobotModule*>(this)->_bounds = nominalBounds(limits);
  return _bounds;
}


const std::map<std::string, std::vector<double> > & PrinterRobotModule::stance() const
{
  const_cast<PrinterRobotModule*>(this)->_stance = halfSittingPose(mb);
  return _stance;
}

}
