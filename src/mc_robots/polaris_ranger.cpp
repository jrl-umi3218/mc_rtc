#include <mc_robots/polaris_ranger.h>

#include <fstream>

namespace mc_robots
{

PolarisRangerRobotModule::PolarisRangerRobotModule()
: RobotModule("/home/hrp2user/jrl/hrp2_drc/hrp2_drc_description/", "polaris_ranger")
{
  halfSitting["steering_joint"] = {0};
  readUrdf("polaris_ranger_interactive", virtualLinks);
}

std::map<std::string, std::pair<std::string, std::string> > PolarisRangerRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const
{
  std::string convexPath = path + "/convex/polaris_ranger/";
  std::map<std::string, std::pair<std::string, std::string> > res;
  for(const auto & f : files)
  {
    res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
  }
  return res;
}

void PolarisRangerRobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
{
  std::string urdfPath = path + "/urdf/" + robotName + ".urdf";
  std::ifstream ifs(urdfPath);
  std::stringstream urdf;
  urdf << ifs.rdbuf();
  mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), true, filteredLinks);
  mb = res.mb;
  mbc = res.mbc;
  mbg = res.mbg;
  limits = res.limits;
  visual_tf = res.visual_tf;
  _collisionTransforms = res.collision_tf;
}

std::map<unsigned int, std::vector<double>> PolarisRangerRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
{
  std::map<unsigned int, std::vector<double>> res;
  for(const auto & j : mb.joints())
  {
    if(j.id() != -1)
    {
      unsigned int k = static_cast<unsigned int>(j.id());
      res[k] = halfSitting.at(j.name());
      for(auto & ji : res[k])
      {
        ji = M_PI*ji/180;
      }
    }
  }
  return res;
}

std::vector< std::map<int, std::vector<double> > > PolarisRangerRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits & limits) const
{
  std::vector< std::map<int, std::vector<double> > > res(0);
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

std::map<std::string, std::pair<std::string, std::string>> PolarisRangerRobotModule::stdCollisionsFiles(const rbd::MultiBody & mb) const
{
  std::map<std::string, std::pair<std::string, std::string>> res;

  res["chassis_back"] = std::pair<std::string, std::string>("chassis", "chassis_back_hull");
  res["chassis_trunk"] = std::pair<std::string, std::string>("chassis", "chassis_trunk_hull");
  res["floor_step"] = std::pair<std::string, std::string>("chassis", "floor_step_hull");
  res["floor"] = std::pair<std::string, std::string>("chassis", "floor_hull");
  res["front_left_rung"] = std::pair<std::string, std::string>("chassis", "front_left_rung_hull");
  res["front_plane"] = std::pair<std::string, std::string>("chassis", "front_plane");
  res["seat_back"] = std::pair<std::string, std::string>("chassis", "seat_back_hull");
  res["seat"] = std::pair<std::string, std::string>("chassis", "seat_hull");
  res["full_seat"] = std::pair<std::string, std::string>("chassis", "full_seat_hull");
  res["top_left_rung"] = std::pair<std::string, std::string>("chassis", "top_left_rung_hull");
  res["wheel"] = std::pair<std::string, std::string>("chassis", "wheel_hull");
  res["little_wheel"] = std::pair<std::string, std::string>("chassis", "little_wheel_hull");
  res["windshield"] = std::pair<std::string, std::string>("chassis", "windshield_hull");
  res["nofeetzone"] = std::pair<std::string, std::string>("chassis", "nofeetzone_hull");
  res["brake"] = std::pair<std::string, std::string>("chassis", "brake_hull");
  res["board"] = std::pair<std::string, std::string>("chassis", "board_hull");
  return res;
}

}
