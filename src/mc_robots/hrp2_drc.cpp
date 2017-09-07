#include "hrp2_drc.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <fstream>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_robots
{

HRP2DRCCommonRobotModule::HRP2DRCCommonRobotModule()
: RobotModule(mc_rtc::HRP2_DRC_DESCRIPTION_PATH, "hrp2_drc", std::string(mc_rtc::HRP2_DRC_DESCRIPTION_PATH) + "/urdf/hrp2drc.urdf")
{
  virtualLinks.push_back("base_link");
  virtualLinks.push_back("l_gripper");
  virtualLinks.push_back("r_gripper");
  virtualLinks.push_back("xtion_link");
  virtualLinks.push_back("LeftHandForceSensor");
  virtualLinks.push_back("RightHandForceSensor");
  virtualLinks.push_back("LeftFootForceSensor");
  virtualLinks.push_back("RightFootForceSensor");

  gripperLinks.push_back("LARM_LINK7");
  gripperLinks.push_back("LHAND_LINK0");
  gripperLinks.push_back("LHAND_LINK1");
  gripperLinks.push_back("LHAND_LINK2");
  gripperLinks.push_back("LHAND_LINK3");
  gripperLinks.push_back("LHAND_LINK4");
  gripperLinks.push_back("RARM_LINK7");
  gripperLinks.push_back("RHAND_LINK0");
  gripperLinks.push_back("RHAND_LINK1");
  gripperLinks.push_back("RHAND_LINK2");
  gripperLinks.push_back("RHAND_LINK3");
  gripperLinks.push_back("RHAND_LINK4");

  _bodySensors.emplace_back("Accelerometer", "CHEST_LINK1", sva::PTransformd(Eigen::Vector3d(-0.13, 0, 0.118)));

  halfSitting["RLEG_JOINT0"] = {0};
  halfSitting["RLEG_JOINT1"] = {0};
  halfSitting["RLEG_JOINT2"] = {-26};
  halfSitting["RLEG_JOINT3"] = {50};
  halfSitting["RLEG_JOINT4"] = {-24};
  halfSitting["RLEG_JOINT5"] = {0};
  halfSitting["LLEG_JOINT0"] = {0};
  halfSitting["LLEG_JOINT1"] = {0};
  halfSitting["LLEG_JOINT2"] = {-26};
  halfSitting["LLEG_JOINT3"] = {50};
  halfSitting["LLEG_JOINT4"] = {-24};
  halfSitting["LLEG_JOINT5"] = {0};
  halfSitting["CHEST_JOINT0"] = {0};
  halfSitting["CHEST_JOINT1"] = {0};
  halfSitting["HEAD_JOINT0"] = {0};
  halfSitting["HEAD_JOINT1"] = {0};
  halfSitting["RARM_JOINT0"] = {45};
  halfSitting["RARM_JOINT1"] = {-20};
  halfSitting["RARM_JOINT2"] = {0};
  halfSitting["RARM_JOINT3"] = {-75};
  halfSitting["RARM_JOINT4"] = {0};
  halfSitting["RARM_JOINT5"] = {0};
  halfSitting["RARM_JOINT6"] = {0};
  halfSitting["RARM_JOINT7"] = {20};
  halfSitting["LARM_JOINT0"] = {45};
  halfSitting["LARM_JOINT1"] = {20};
  halfSitting["LARM_JOINT2"] = {0};
  halfSitting["LARM_JOINT3"] = {-75};
  halfSitting["LARM_JOINT4"] = {0};
  halfSitting["LARM_JOINT5"] = {0};
  halfSitting["LARM_JOINT6"] = {0};
  halfSitting["LARM_JOINT7"] = {20};
  halfSitting["RHAND_JOINT0"] = {-20};
  halfSitting["RHAND_JOINT1"] = {20};
  halfSitting["RHAND_JOINT2"] = {-20};
  halfSitting["RHAND_JOINT3"] = {20};
  halfSitting["RHAND_JOINT4"] = {-20};
  halfSitting["LHAND_JOINT0"] = {-20};
  halfSitting["LHAND_JOINT1"] = {20};
  halfSitting["LHAND_JOINT2"] = {-20};
  halfSitting["LHAND_JOINT3"] = {20};
  halfSitting["LHAND_JOINT4"] = {-20};

  _forceSensors.push_back(mc_rbdyn::ForceSensor("RightFootForceSensor", "RLEG_LINK5", sva::PTransformd(Eigen::Vector3d(0, 0, -0.1050))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftFootForceSensor", "LLEG_LINK5", sva::PTransformd(Eigen::Vector3d(0, 0, -0.1050))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RightHandForceSensor", "RARM_LINK6", sva::PTransformd(Eigen::Vector3d(0, 0, -0.087))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "LARM_LINK6", sva::PTransformd(Eigen::Vector3d(0, 0, -0.087))));

  _minimalSelfCollisions = {
    mc_rbdyn::Collision("LARM_LINK3", "BODY", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("LARM_LINK4", "BODY", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("LARM_LINK5", "BODY", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("RARM_LINK3", "BODY", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("RARM_LINK4", "BODY", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("RARM_LINK5", "BODY", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("RARM_LINK3", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("RARM_LINK4", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("RARM_LINK5", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("RARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("RARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("LARM_LINK3", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("LARM_LINK4", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK0", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("LARM_LINK4", "CHEST_LINK1", 0.05, 0.01, 0.),
    mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK1", 0.05, 0.01, 0.)
  };

  _commonSelfCollisions = _minimalSelfCollisions;

  _grippers = {
    {"l_gripper", {"LARM_JOINT7"}, false},
    {"r_gripper", {"RARM_JOINT7"}, false}
  };

  _ref_joint_order = {
  "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
  "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5",
  "CHEST_JOINT0", "CHEST_JOINT1", "HEAD_JOINT0", "HEAD_JOINT1",
  "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6", "RARM_JOINT7",
  "LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6", "LARM_JOINT7",
  "RHAND_JOINT0", "RHAND_JOINT1", "RHAND_JOINT2", "RHAND_JOINT3", "RHAND_JOINT4",
  "LHAND_JOINT0", "LHAND_JOINT1", "LHAND_JOINT2", "LHAND_JOINT3", "LHAND_JOINT4"
  };

  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.773}};
}

std::map<std::string, std::pair<std::string, std::string> > HRP2DRCCommonRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const
{
  std::string convexPath = path + "/convex/hrp2_drc/";
  std::map<std::string, std::pair<std::string, std::string> > res;
  for(const auto & f : files)
  {
    bfs::path fpath(convexPath + f.second.second + "-ch.txt");
    if(bfs::exists(fpath))
    {
      res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
    }
  }
  return res;
}

void HRP2DRCCommonRobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
{
  std::string urdfPath = path + "/urdf/" + robotName + ".urdf";
  std::ifstream ifs(urdfPath);
  if(ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false, filteredLinks);
    mb = res.mb;
    mbc = res.mbc;
    mbg = res.mbg;
    limits = res.limits;

    _visual = res.visual;
    _collisionTransforms = res.collision_tf;
  }
  else
  {
    LOG_ERROR("Could not open HRP2DRC model at " << urdfPath)
    throw("HRP2DRC model not found");
  }
}

std::map<std::string, std::vector<double> > HRP2DRCCommonRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
{
  std::map<std::string, std::vector<double> > res;
  for(const auto & j : mb.joints())
  {
    if(j.name() != "Root")
    {
      if(halfSitting.count(j.name()))
      {
        res[j.name()] = halfSitting.at(j.name());
        for(auto & ji : res[j.name()])
        {
          ji = M_PI*ji/180;
        }
      }
      else
      {
        res[j.name()] = j.zeroParam();
      }
    }
  }
  return res;
}

std::vector< std::map<std::string, std::vector<double> > > HRP2DRCCommonRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits & limits) const
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

std::map<std::string, std::pair<std::string, std::string>> HRP2DRCCommonRobotModule::stdCollisionsFiles(const rbd::MultiBody & mb) const
{
  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & b : mb.bodies())
  {
    res[b.name()] = std::pair<std::string, std::string>(b.name(), b.name());
  }
  res["RARM_LINK7"] = std::pair<std::string, std::string>("RARM_LINK7", "RARM_DRC_LINK7");
  res["LARM_LINK7"] = std::pair<std::string, std::string>("LARM_LINK7", "LARM_DRC_LINK7");
  res["RARM_LINK6_sub1"] = std::pair<std::string, std::string>("RARM_LINK6", "RARM_LINK6_lower");
  res["LARM_LINK6_sub1"] = std::pair<std::string, std::string>("LARM_LINK6", "LARM_LINK6_lower");
  res["RARM_LINK6"] = std::pair<std::string, std::string>("RARM_LINK6", "r_wrist_closed");
  res["LARM_LINK6"] = std::pair<std::string, std::string>("LARM_LINK6", "l_wrist_closed");
  res["CHEST_LINK1_FULL"] = std::pair<std::string, std::string>("CHEST_LINK1", "CHEST_LINK1_FULL");
  return res;
}

HRP2DRCRobotModule::HRP2DRCRobotModule()
{
  filteredLinks = virtualLinks;
  for(const auto & gl : gripperLinks)
  {
    filteredLinks.push_back(gl);
  }
  readUrdf("hrp2drc", filteredLinks);

  _springs.springsBodies = {"LLEG_LINK5", "RLEG_LINK5"};

  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);
  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);
}

const std::map<std::string, std::pair<std::string, std::string> > & HRP2DRCRobotModule::convexHull() const
{
  return _convexHull;
}

const std::vector< std::map<std::string, std::vector<double> > > & HRP2DRCRobotModule::bounds() const
{
  return _bounds;
}

const std::map<std::string, std::vector<double> > & HRP2DRCRobotModule::stance() const
{
  return _stance;
}

HRP2DRCGripperRobotModule::HRP2DRCGripperRobotModule()
{
  filteredLinks = virtualLinks;
  readUrdf("hrp2drc", filteredLinks);

  _springs.springsBodies = {"LLEG_LINK5", "RLEG_LINK5"};
  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);
  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);
}

const std::map<std::string, std::pair<std::string, std::string> > & HRP2DRCGripperRobotModule::convexHull() const
{
  return _convexHull;
}

const std::vector< std::map<std::string, std::vector<double> > > & HRP2DRCGripperRobotModule::bounds() const
{
  return _bounds;
}

const std::map<std::string, std::vector<double> > & HRP2DRCGripperRobotModule::stance() const
{
  return _stance;
}

}
