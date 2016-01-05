#include "hrp4.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <fstream>

namespace mc_robots
{

  HRP4CommonRobotModule::HRP4CommonRobotModule()
    : RobotModule(mc_rtc::HRP4_DESCRIPTION_PATH, "hrp4")
  {
    virtualLinks.push_back("base_link");
    virtualLinks.push_back("r_sole");
    virtualLinks.push_back("l_sole");
    virtualLinks.push_back("Accelerometer");
    virtualLinks.push_back("Gyro");
    virtualLinks.push_back("RightFootForceSensor");
    virtualLinks.push_back("LeftFootForceSensor");
    virtualLinks.push_back("LeftHandForceSensor");
    virtualLinks.push_back("RightHandForceSensor");
    virtualLinks.push_back("l_gripper");
    virtualLinks.push_back("r_gripper");
    virtualLinks.push_back("gaze");
    virtualLinks.push_back("r_gripper_sensor");

    gripperLinks.push_back("R_HAND_J0_LINK");
    gripperLinks.push_back("R_HAND_J1_LINK");
    gripperLinks.push_back("R_F22_LINK");
    gripperLinks.push_back("R_F23_LINK");
    gripperLinks.push_back("R_F32_LINK");
    gripperLinks.push_back("R_F33_LINK");
    gripperLinks.push_back("R_F42_LINK");
    gripperLinks.push_back("R_F43_LINK");
    gripperLinks.push_back("R_F52_LINK");
    gripperLinks.push_back("R_F53_LINK");
    gripperLinks.push_back("L_HAND_J0_LINK");
    gripperLinks.push_back("L_HAND_J1_LINK");
    gripperLinks.push_back("L_F22_LINK");
    gripperLinks.push_back("L_F23_LINK");
    gripperLinks.push_back("L_F32_LINK");
    gripperLinks.push_back("L_F33_LINK");
    gripperLinks.push_back("L_F42_LINK");
    gripperLinks.push_back("L_F43_LINK");
    gripperLinks.push_back("L_F52_LINK");
    gripperLinks.push_back("L_F53_LINK");

    _accelerometerBody = "body";

    halfSitting["R_HIP_Y"] = { 0 }; //0
    halfSitting["R_HIP_R"] = { -0.76 }; //-0.0132645
    halfSitting["R_HIP_P"] = { -22.02 }; //-0.38432149
    halfSitting["R_KNEE_P"] = { 41.29 }; //0.72064644
    halfSitting["R_ANKLE_P"] = { -18.75 }; //-0.32724923
    halfSitting["R_ANKLE_R"] = { -0.45 }; //-0.00785398
    halfSitting["L_HIP_Y"] = { 0 }; //0
    halfSitting["L_HIP_R"] = { 1.15 }; //0.02007129
    halfSitting["L_HIP_P"] = { -21.89 }; //-0.38205257
    halfSitting["L_KNEE_P"] = { 41.21 }; //0.71925017
    halfSitting["L_ANKLE_P"] = { -18.74 }; //-0.3270747
    halfSitting["L_ANKLE_R"] = { -1.1 }; //-0.01919862
    halfSitting["CHEST_P"] = { 8 }; //0.13962634
    halfSitting["CHEST_Y"] = { 0 }; //0
    halfSitting["NECK_Y"] = { 0 }; //0
    halfSitting["NECK_P"] = { 0 }; //0
    halfSitting["R_SHOULDER_P"] = { -3 }; //-0.05235988
    halfSitting["R_SHOULDER_R"] = { -10 }; //-0.17453292
    halfSitting["R_SHOULDER_Y"] = { 0 }; //0.
    halfSitting["R_ELBOW_P"] = { -30 }; //-0.52359877
    halfSitting["R_WRIST_Y"] = { 0 }; //0
    halfSitting["R_WRIST_P"] = { 0 }; //0
    halfSitting["R_WRIST_R"] = { 0 }; //0
    halfSitting["R_HAND_J0"] = { 0 }; //0
    halfSitting["R_HAND_J1"] = { 0 }; //0
    halfSitting["L_SHOULDER_P"] = { -3 }; //-0.05235988
    halfSitting["L_SHOULDER_R"] = { 10 }; //0.17453292
    halfSitting["L_SHOULDER_Y"] = { 0 }; //0
    halfSitting["L_ELBOW_P"] = { -30 }; //-0.52359877
    halfSitting["L_WRIST_Y"] = { 0 }; //0
    halfSitting["L_WRIST_P"] = { 0 }; //0
    halfSitting["L_WRIST_R"] = { 0 }; //0
    halfSitting["L_HAND_J0"] = { 0 }; //0
    halfSitting["L_HAND_J1"] = { 0 }; //0
    halfSitting["R_F22"] = { 0 }; //0
    halfSitting["R_F23"] = { 0 }; //0
    halfSitting["R_F32"] = { 0 }; //0
    halfSitting["R_F33"] = { 0 }; //0
    halfSitting["R_F42"] = { 0 }; //0
    halfSitting["R_F43"] = { 0 }; //0
    halfSitting["R_F52"] = { 0 }; //0
    halfSitting["R_F53"] = { 0 }; //0
    halfSitting["L_F22"] = { 0 }; //0
    halfSitting["L_F23"] = { 0 }; //0
    halfSitting["L_F32"] = { 0 }; //0
    halfSitting["L_F33"] = { 0 }; //0
    halfSitting["L_F42"] = { 0 }; //0
    halfSitting["L_F43"] = { 0 }; //0
    halfSitting["L_F52"] = { 0 }; //0
    halfSitting["L_F53"] = { 0 }; //0
    halfSitting["R_FOOT"] = {};
    halfSitting["L_FOOT"] = {};

    _forceSensors.push_back(mc_rbdyn::ForceSensor("RightFootForceSensor", "R_ANKLE_R_LINK", sva::PTransformd(Eigen::Vector3d(0, 0, -0.093))));
    _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftFootForceSensor", "L_ANKLE_R_LINK", sva::PTransformd(Eigen::Vector3d(0, 0, -0.093))));
    Eigen::Matrix3d R; R << 0, -1, 0, -1, 0, 0, 0, 0, -1; // rpy="3.14159 0 -1.57079"
    _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "l_wrist", sva::PTransformd(R, Eigen::Vector3d(0, 0, -0.04435))));
    _forceSensors.push_back(mc_rbdyn::ForceSensor("RightHandForceSensor", "r_wrist", sva::PTransformd(R, Eigen::Vector3d(0, 0, -0.04435))));
  }

  std::map<std::string, std::pair<std::string, std::string> > HRP4CommonRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const
  {
    std::string convexPath = path + "/convex/";
    std::map<std::string, std::pair<std::string, std::string> > res;
    for (const auto & f : files)
    {
      res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
    }
    return res;
  }

  void HRP4CommonRobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
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
      visual_tf = res.visual_tf;
      _collisionTransforms = res.collision_tf;
    }
    else
    {
      LOG_ERROR("Could not open HRP4 model at " << urdfPath)
      throw("Failed to open HRP4 model");
    }
  }

  std::map<unsigned int, std::vector<double>> HRP4CommonRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
  {
    std::map<unsigned int, std::vector<double>> res;
    for (const auto & j : mb.joints())
    {
      if (j.id() != -1)
      {
        unsigned int k = static_cast<unsigned int>(j.id());
        res[k] = halfSitting.at(j.name());
        for (auto & ji : res[k])
        {
          ji = M_PI*ji / 180;
        }
      }
    }
    return res;
  }

  std::vector< std::map<int, std::vector<double> > > HRP4CommonRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits & limits) const
  {
    std::vector< std::map<int, std::vector<double> > > res(0);
    res.push_back(limits.lower);
    res.push_back(limits.upper);
    {
      auto mvelocity = limits.velocity;
      for (auto & mv : mvelocity)
      {
        for (auto & mvi : mv.second)
        {
          mvi = -mvi;
        }
      }
      res.push_back(mvelocity);
    }
    res.push_back(limits.velocity);
    {
      auto mtorque = limits.torque;
      for (auto & mt : mtorque)
      {
        for (auto & mti : mt.second)
        {
          mti = -mti;
        }
      }
      res.push_back(mtorque);
    }
    res.push_back(limits.torque);
    return res;
  }

  std::map<std::string, std::pair<std::string, std::string>> HRP4CommonRobotModule::stdCollisionsFiles(const rbd::MultiBody & mb) const
  {
    std::map<std::string, std::pair<std::string, std::string>> res;
    //TODO: add convex hulls
    //for (const auto & b : mb.bodies())
    //{
    //  res[b.name()] = std::pair<std::string, std::string>(b.name(), b.name());
    //}
    //res["RARM_LINK7"] = std::pair<std::string, std::string>("RARM_LINK7", "RARM_DRC_LINK7");
    //res["LARM_LINK7"] = std::pair<std::string, std::string>("LARM_LINK7", "LARM_DRC_LINK7");
    //res["RARM_LINK6_sub1"] = std::pair<std::string, std::string>("RARM_LINK6", "RARM_LINK6_lower");
    //res["LARM_LINK6_sub1"] = std::pair<std::string, std::string>("LARM_LINK6", "LARM_LINK6_lower");
    //res["RLEG_LINK5"] = std::pair<std::string, std::string>("RLEG_LINK5", "RLEG_LINK5-2");
    //res["LLEG_LINK5"] = std::pair<std::string, std::string>("LLEG_LINK5", "LLEG_LINK5-2");
    //res["RARM_LINK6"] = std::pair<std::string, std::string>("RARM_LINK6", "r_wrist_closed");
    //res["LARM_LINK6"] = std::pair<std::string, std::string>("LARM_LINK6", "l_wrist_closed");
    //res["CHEST_LINK1_FULL"] = std::pair<std::string, std::string>("CHEST_LINK1", "CHEST_LINK1_FULL");
    return res;
  }

  HRP4NoHandRobotModule::HRP4NoHandRobotModule()
  {
    filteredLinks = virtualLinks;
    for (const auto & gl : gripperLinks)
    {
      filteredLinks.push_back(gl);
    }
    readUrdf("hrp4", filteredLinks);

    _springs.springsBodies = { "l_ankle", "r_ankle" }; //TODO: check these are the correct bodies
  }

  const std::map<std::string, std::pair<std::string, std::string> > & HRP4NoHandRobotModule::convexHull() const
  {
    auto fileByBodyName = stdCollisionsFiles(mb);
    const_cast<HRP4NoHandRobotModule*>(this)->_convexHull = getConvexHull(fileByBodyName);
    return _convexHull;
  }

  const std::vector< std::map<int, std::vector<double> > > & HRP4NoHandRobotModule::bounds() const
  {
    const_cast<HRP4NoHandRobotModule*>(this)->_bounds = nominalBounds(limits);
    return _bounds;
  }

  const std::map< unsigned int, std::vector<double> > & HRP4NoHandRobotModule::stance() const
  {
    const_cast<HRP4NoHandRobotModule*>(this)->_stance = halfSittingPose(mb);
    return _stance;
  }

  HRP4WithHandRobotModule::HRP4WithHandRobotModule()
  {
    filteredLinks = virtualLinks;
    readUrdf("hrp4", filteredLinks);

    _springs.springsBodies = { "l_ankle", "r_ankle" }; //TODO: check these are the correct bodies
  }

  const std::map<std::string, std::pair<std::string, std::string> > & HRP4WithHandRobotModule::convexHull() const
  {
    auto fileByBodyName = stdCollisionsFiles(mb);
    const_cast<HRP4WithHandRobotModule*>(this)->_convexHull = getConvexHull(fileByBodyName);
    return _convexHull;
  }

  const std::vector< std::map<int, std::vector<double> > > & HRP4WithHandRobotModule::bounds() const
  {
    const_cast<HRP4WithHandRobotModule*>(this)->_bounds = nominalBounds(limits);
    return _bounds;
  }

  const std::map< unsigned int, std::vector<double> > & HRP4WithHandRobotModule::stance() const
  {
    const_cast<HRP4WithHandRobotModule*>(this)->_stance = halfSittingPose(mb);
    return _stance;
  }

}
