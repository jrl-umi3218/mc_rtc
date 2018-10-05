#include "polaris_ranger.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <fstream>

namespace mc_robots
{

PolarisRangerRobotModule::PolarisRangerRobotModule(bool is_interactive)
: RobotModule(mc_rtc::HRP2_DRC_DESCRIPTION_PATH, "polaris_ranger")
{
  halfSitting["POLARIS"] = {};
  halfSitting["front_left_steering_joint"] = {};
  halfSitting["front_left_wheel_joint"] = {};
  halfSitting["front_right_steering_joint"] = {};
  halfSitting["front_right_wheel_joint"] = {};
  halfSitting["rear_left_wheel_joint"] = {};
  halfSitting["rear_right_wheel_joint"] = {};
  halfSitting["gas_joint"] = {};
  halfSitting["brake_joint"] = {};
  halfSitting["hand_brake_joint"] = {};
  halfSitting["FNR_switch_joint"] = {};
  halfSitting["lazy_susan"] = {};

  if(is_interactive)
  {
    halfSitting["steering_joint"] = {0};
    halfSitting["adjust_steering_wheel"] = {};
    readUrdf("polaris_ranger_interactive", virtualLinks);
  }
  else
  {
    halfSitting["steering_joint"] = {};
    halfSitting["top_left_frame_joint"] = {};
    readUrdf("polaris_ranger", virtualLinks);
  }
  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);
  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);
}

std::map<std::string, std::pair<std::string, std::string>> PolarisRangerRobotModule::getConvexHull(
    const std::map<std::string, std::pair<std::string, std::string>> & files) const
{
  std::string convexPath = path + "/convex/polaris_ranger/";
  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & f : files)
  {
    res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
  }
  return res;
}

void PolarisRangerRobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
{
  urdf_path = path + "/urdf/" + robotName + ".urdf";
  std::ifstream ifs(urdf_path);
  if(ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false, filteredLinks);
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
    LOG_ERROR("Could not open PolarisRanger model at " << urdf_path)
    LOG_ERROR_AND_THROW(std::runtime_error, "Failed to open PolarisRanger model")
  }
}

std::map<std::string, std::vector<double>> PolarisRangerRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
{
  std::map<std::string, std::vector<double>> res;
  for(const auto & j : mb.joints())
  {
    if(j.name() != "Root")
    {
      res[j.name()] = halfSitting.at(j.name());
      for(auto & ji : res[j.name()])
      {
        ji = M_PI * ji / 180;
      }
    }
  }
  return res;
}

std::vector<std::map<std::string, std::vector<double>>> PolarisRangerRobotModule::nominalBounds(
    const mc_rbdyn_urdf::Limits & limits) const
{
  std::vector<std::map<std::string, std::vector<double>>> res(0);
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

std::map<std::string, std::pair<std::string, std::string>> PolarisRangerRobotModule::stdCollisionsFiles(
    const rbd::MultiBody & /*mb*/) const
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
  res["top_left_rung_lower"] = std::pair<std::string, std::string>("top_left_frame", "top_left_rung_lower_hull");
  res["top_left_frame"] = std::pair<std::string, std::string>("top_left_frame", "top_left_frame");
  res["wheel"] = std::pair<std::string, std::string>("chassis", "wheel_hull");
  res["little_wheel"] = std::pair<std::string, std::string>("chassis", "little_wheel_hull");
  res["windshield"] = std::pair<std::string, std::string>("chassis", "windshield_hull");
  res["nofeetzone"] = std::pair<std::string, std::string>("chassis", "nofeetzone_hull");
  res["brake"] = std::pair<std::string, std::string>("chassis", "brake_hull");
  res["board"] = std::pair<std::string, std::string>("chassis", "board_hull");
  res["lateral_front"] = std::pair<std::string, std::string>("chassis", "lateral_front_hull");
  res["chassis"] = std::pair<std::string, std::string>("chassis", "chassis");
  return res;
}

const std::map<std::string, std::pair<std::string, std::string>> & PolarisRangerRobotModule::convexHull() const
{
  return _convexHull;
}

const std::vector<std::map<std::string, std::vector<double>>> & PolarisRangerRobotModule::bounds() const
{
  return _bounds;
}

const std::map<std::string, std::vector<double>> & PolarisRangerRobotModule::stance() const
{
  return _stance;
}

} // namespace mc_robots
