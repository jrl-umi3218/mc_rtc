#include <mc_robots/hrp2_drc.h>

#include <mc_rtc/config.h>

#include <fstream>

namespace mc_robots
{

HRP2DRCCommonRobotModule::HRP2DRCCommonRobotModule()
: RobotModule(mc_rtc::HRP2_DRC_DESCRIPTION_PATH, "hrp2_drc")
{
  virtualLinks.push_back("base_link");
  virtualLinks.push_back("Accelerometer");
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

  //_forceSensors.push_back(mc_rbdyn::ForceSensor("RightFootForceSensor", "RLEG_LIN5", sva::PTransformd(Eigen::Vector3d(0, 0, -0.105))));
  //_forceSensors.push_back(mc_rbdyn::ForceSensor("LeftFootForceSensor", "LLEG_LINK5", sva::PTransformd(Eigen::Vector3d(0, 0, -0.105))));
  //_forceSensors.push_back(mc_rbdyn::ForceSensor("RightHandForceSensor", "RARM_LINK6", sva::PTransformd(Eigen::Vector3d(0.005, 0, -0.05925))));
  //_forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "LARM_LINK6", sva::PTransformd(Eigen::Vector3d(0.005, 0, -0.05925))));

  _accelerometerBody = "CHEST_LINK1";

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

  _forceSensors.push_back(mc_rbdyn::ForceSensor("RightFootForceSensor", "RLEG_LINK5", sva::PTransformd(Eigen::Vector3d(0, 0, -0.195))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftFootForceSensor", "LLEG_LINK5", sva::PTransformd(Eigen::Vector3d(0, 0, -0.195))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RightHandForceSensor", "RARM_LINK6", sva::PTransformd(Eigen::Vector3d(0, 0, -0.087))));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "LARM_LINK6", sva::PTransformd(Eigen::Vector3d(0, 0, -0.087))));
}

std::map<std::string, std::pair<std::string, std::string> > HRP2DRCCommonRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const
{
  std::string convexPath = path + "/convex/hrp2_drc/";
  std::map<std::string, std::pair<std::string, std::string> > res;
  for(const auto & f : files)
  {
    res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
  }
  return res;
}

void HRP2DRCCommonRobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
{
  std::string urdfPath = path + "urdf/" + robotName + ".urdf";
  std::cout  << "URDF PATH: "<< urdfPath << std::endl;
  std::ifstream ifs(urdfPath);
  std::stringstream urdf;
  urdf << ifs.rdbuf();
  mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false, filteredLinks);
  mb = res.mb;
  mbc = res.mbc;
  mbg = res.mbg;
  limits = res.limits;
  visual_tf = res.visual_tf;
  _collisionTransforms = res.collision_tf;
}

std::map<unsigned int, std::vector<double>> HRP2DRCCommonRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
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

std::vector< std::map<int, std::vector<double> > > HRP2DRCCommonRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits & limits) const
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
  res["RLEG_LINK5"] = std::pair<std::string, std::string>("RLEG_LINK5", "RLEG_LINK5-2");
  res["LLEG_LINK5"] = std::pair<std::string, std::string>("LLEG_LINK5", "LLEG_LINK5-2");
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
}

const std::map<std::string, std::pair<std::string, std::string> > & HRP2DRCRobotModule::convexHull() const
{
  auto fileByBodyName = stdCollisionsFiles(mb);
  const_cast<HRP2DRCRobotModule*>(this)->_convexHull = getConvexHull(fileByBodyName);
  return _convexHull;
}

const std::vector< std::map<int, std::vector<double> > > & HRP2DRCRobotModule::bounds() const
{
  const_cast<HRP2DRCRobotModule*>(this)->_bounds = nominalBounds(limits);
  return _bounds;
}

const std::map< unsigned int, std::vector<double> > & HRP2DRCRobotModule::stance() const
{
  const_cast<HRP2DRCRobotModule*>(this)->_stance = halfSittingPose(mb);
  return _stance;
}

HRP2DRCGripperRobotModule::HRP2DRCGripperRobotModule()
{
  filteredLinks = virtualLinks;
  readUrdf("hrp2drc", filteredLinks);

  _springs.springsBodies = {"LLEG_LINK5", "RLEG_LINK5"};
}

const std::map<std::string, std::pair<std::string, std::string> > & HRP2DRCGripperRobotModule::convexHull() const
{
  auto fileByBodyName = stdCollisionsFiles(mb);
  const_cast<HRP2DRCGripperRobotModule*>(this)->_convexHull = getConvexHull(fileByBodyName);
  return _convexHull;
}

const std::vector< std::map<int, std::vector<double> > > & HRP2DRCGripperRobotModule::bounds() const
{
  const_cast<HRP2DRCGripperRobotModule*>(this)->_bounds = nominalBounds(limits);
  return _bounds;
}

const std::map< unsigned int, std::vector<double> > & HRP2DRCGripperRobotModule::stance() const
{
  const_cast<HRP2DRCGripperRobotModule*>(this)->_stance = halfSittingPose(mb);
  return _stance;
}

}
