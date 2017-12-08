#include "hrp4.h"

#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <boost/algorithm/string.hpp>
#include <fstream>

namespace mc_robots
{

  HRP4CommonRobotModule::HRP4CommonRobotModule()
    : RobotModule(mc_rtc::HRP4_DESCRIPTION_PATH, "hrp4")
  {
    rsdf_dir = path + "/rsdf";
    calib_dir = path + "/calib";

    virtualLinks.push_back("base_link");
    virtualLinks.push_back("Accelerometer");
    virtualLinks.push_back("Gyro");
    virtualLinks.push_back("RightFootForceSensor");
    virtualLinks.push_back("LeftFootForceSensor");
    virtualLinks.push_back("LeftHandForceSensor");
    virtualLinks.push_back("RightHandForceSensor");
    virtualLinks.push_back("gaze");
    virtualLinks.push_back("r_gripper_sensor");
    virtualLinks.push_back("xtion_link");
    virtualLinks.push_back("r_gripper");
    virtualLinks.push_back("l_gripper");
    virtualLinks.push_back("r_sole");
    virtualLinks.push_back("l_sole");

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

    _bodySensors.emplace_back("Accelerometer", "body", sva::PTransformd(Eigen::Vector3d(-0.0325, 0, 0.1095)));

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
    /* Virtual joints */
    halfSitting["L_FOOT"] = {};
    halfSitting["R_FOOT"] = {};

    _forceSensors.push_back(mc_rbdyn::ForceSensor("RightFootForceSensor", "r_ankle", sva::PTransformd(Eigen::Vector3d(0, 0, -0.093))));
    _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftFootForceSensor", "l_ankle", sva::PTransformd(Eigen::Vector3d(0, 0, -0.093))));
    Eigen::Matrix3d R; R << 0, -1, 0, -1, 0, 0, 0, 0, -1; // rpy="3.14159 0 -1.57079"
    _forceSensors.push_back(mc_rbdyn::ForceSensor("RightHandForceSensor", "r_wrist", sva::PTransformd(R, Eigen::Vector3d(0, 0, -0.04435))));
    _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "l_wrist", sva::PTransformd(R, Eigen::Vector3d(0, 0, -0.04435))));

    _minimalSelfCollisions = {
      mc_rbdyn::Collision("torso", "L_SHOULDER_Y_LINK", 0.02, 0.001, 0.),
      mc_rbdyn::Collision("body", "L_ELBOW_P_LINK", 0.05, 0.001, 0.),
      mc_rbdyn::Collision("torso", "R_SHOULDER_Y_LINK", 0.02, 0.001, 0.),
      mc_rbdyn::Collision("body", "R_ELBOW_P_LINK", 0.05, 0.001, 0.),
      mc_rbdyn::Collision("l_wrist", "L_HIP_P_LINK", 0.07, 0.05, 0.),
      mc_rbdyn::Collision("r_wrist", "R_HIP_P_LINK", 0.07, 0.05, 0.),
      mc_rbdyn::Collision("r_wrist_sub0", "R_WRIST_Y_LINK_sub0", 0.005, 0.001, 0.),
      mc_rbdyn::Collision("r_wrist_sub1", "R_WRIST_Y_LINK_sub0", 0.005, 0.001, 0.),
      mc_rbdyn::Collision("l_wrist_sub0", "L_WRIST_Y_LINK_sub0", 0.005, 0.001, 0.),
      mc_rbdyn::Collision("l_wrist_sub1", "L_WRIST_Y_LINK_sub0", 0.005, 0.001, 0.),
      mc_rbdyn::Collision("R_HIP_P_LINK", "body", 0.02, 0.01, 0.),
      mc_rbdyn::Collision("L_HIP_P_LINK", "body", 0.02, 0.01, 0.)
    };

    _commonSelfCollisions = _minimalSelfCollisions;
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("L_HIP_P_LINK", "body", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("L_HIP_P_LINK", "R_HIP_P_LINK", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("L_HIP_P_LINK", "R_KNEE_P_LINK", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("R_HIP_P_LINK", "L_KNEE_P_LINK", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("L_KNEE_P_LINK", "R_KNEE_P_LINK", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("l_ankle", "r_ankle", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("l_ankle", "R_KNEE_P_LINK", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("l_ankle", "R_HIP_P_LINK", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("r_ankle", "L_KNEE_P_LINK", 0.02, 0.01, 0.));
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("r_ankle", "L_HIP_P_LINK", 0.02, 0.01, 0.));

    _grippers = {
      {"l_gripper", {"L_HAND_J0", "L_HAND_J1"}, false},
      {"r_gripper", {"R_HAND_J0", "R_HAND_J1"}, true}
    };

    _ref_joint_order = {
      "R_HIP_Y", "R_HIP_R", "R_HIP_P", "R_KNEE_P", "R_ANKLE_P", "R_ANKLE_R",
      "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R",
      "CHEST_P", "CHEST_Y", "NECK_Y", "NECK_P",
      "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y",
      "R_ELBOW_P",
      "R_WRIST_Y", "R_WRIST_P", "R_WRIST_R",
      "R_HAND_J0", "R_HAND_J1",
      "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y",
      "L_ELBOW_P",
      "L_WRIST_Y", "L_WRIST_P", "L_WRIST_R",
      "L_HAND_J0", "L_HAND_J1"
    };

    _default_attitude = {{1., 0., 0., 0., 0., 0., 0.79216}};
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

      _visual = res.visual;
      _collisionTransforms = res.collision_tf;
    }
    else
    {
      LOG_ERROR("Could not open HRP4 model at " << urdfPath)
      throw("Failed to open HRP4 model");
    }
  }

  std::map<std::string, std::vector<double>> HRP4CommonRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
  {
    std::map<std::string, std::vector<double>> res;
    for (const auto & j : mb.joints())
    {
      if(halfSitting.count(j.name()))
      {
        res[j.name()] = halfSitting.at(j.name());
        for (auto & ji : res[j.name()])
        {
          ji = M_PI*ji / 180;
        }
      }
      else if(j.name() != "Root" && j.dof() > 0)
      {
        LOG_WARNING("Joint " << j.name() << " has " << j.dof() << " dof, but is not part of half sitting posture.");
      }
    }
    return res;
  }

  std::vector< std::map<std::string, std::vector<double> > > HRP4CommonRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits & limits) const
  {
    std::vector< std::map<std::string, std::vector<double> > > res(0);
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
    for(const auto & b : mb.bodies())
    {
      // Filter out virtual links without convex files
      if(std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
      {
        res[b.name()] = {b.name(), boost::algorithm::replace_first_copy(b.name(), "_LINK", "")};
      }
    }

    auto addBody = [&res](const std::string & body, const std::string & file)
    {
      res[body] = {body, file};
    };
    addBody("body", "WAIST_LINK");
    addBody("torso", "CHEST_Y");

    addBody("R_HIP_Y_LINK", "HIP_Y");
    addBody("R_HIP_R_LINK", "CHEST_P");
    addBody("R_ANKLE_P_LINK", "L_ANKLE_P");
    addBody("r_ankle", "R_FOOT");

    addBody("L_HIP_Y_LINK", "HIP_Y");
    addBody("L_HIP_R_LINK", "CHEST_P");
    addBody("l_ankle", "L_FOOT");

    addBody("CHEST_P_LINK", "CHEST");

    addBody("R_SHOULDER_Y_LINK", "SHOULDER_Y");
    addBody("R_ELBOW_P_LINK", "ELBOW_P");
    addBody("R_WRIST_P_LINK", "WRIST_P");
    addBody("r_wrist", "R_WRIST_R");

    auto finger = [&addBody](const std::string & prefix)
    {
      addBody(prefix + "_HAND_J0_LINK", prefix + "_THUMB");
      addBody(prefix + "_HAND_J1_LINK", prefix + "_F1");
      for(unsigned int i = 2; i < 6; ++i)
      {
        std::stringstream key1;
        key1 << prefix << "_F" << i << "2_LINK";
        std::stringstream key2;
        key2 << prefix << "_F" << i << "3_LINK";
        addBody(key1.str(), "F2");
        addBody(key2.str(), "F3");
      }
    };
    finger("R");
    finger("L");

    addBody("L_SHOULDER_Y_LINK", "SHOULDER_Y");
    addBody("L_ELBOW_P_LINK", "ELBOW_P");
    addBody("L_WRIST_P_LINK", "WRIST_P");
    addBody("l_wrist", "L_WRIST_R");

    auto addWristSubConvex = [&res](const std::string & prefix)
    {
      std::string wristY = prefix + "_WRIST_Y_LINK";
      std::string wristR = boost::algorithm::to_lower_copy(prefix) + "_wrist";
      res[wristY + "_sub0"] = {wristY, prefix + "_WRIST_Y_sub0"};
      res[wristR + "_sub0"] = {wristR, prefix + "_WRIST_R_sub0"};
      res[wristR + "_sub1"] = {wristR, prefix + "_WRIST_R_sub1"};
    };
    addWristSubConvex("L");
    addWristSubConvex("R");

    return res;
  }

  HRP4NoHandRobotModule::HRP4NoHandRobotModule()
  {
    for (const auto & gl : gripperLinks)
    {
      filteredLinks.push_back(gl);
    }
    readUrdf("hrp4", filteredLinks);

    _springs.springsBodies = { "l_ankle", "r_ankle" }; //TODO: check these are the correct bodies

    auto fileByBodyName = stdCollisionsFiles(mb);
    _convexHull = getConvexHull(fileByBodyName);
    _bounds = nominalBounds(limits);
    _stance = halfSittingPose(mb);
  }

  const std::map<std::string, std::pair<std::string, std::string> > & HRP4NoHandRobotModule::convexHull() const
  {
    return _convexHull;
  }

  const std::vector< std::map<std::string, std::vector<double> > > & HRP4NoHandRobotModule::bounds() const
  {
    return _bounds;
  }

  const std::map<std::string, std::vector<double> > & HRP4NoHandRobotModule::stance() const
  {
    return _stance;
  }

  HRP4WithHandRobotModule::HRP4WithHandRobotModule()
  {
    readUrdf("hrp4", filteredLinks);

    _springs.springsBodies = { "l_ankle", "r_ankle" }; //TODO: check these are the correct bodies
    auto fileByBodyName = stdCollisionsFiles(mb);
    _convexHull = getConvexHull(fileByBodyName);
    _bounds = nominalBounds(limits);
    _stance = halfSittingPose(mb);
  }

  const std::map<std::string, std::pair<std::string, std::string> > & HRP4WithHandRobotModule::convexHull() const
  {
    return _convexHull;
  }

  const std::vector< std::map<std::string, std::vector<double> > > & HRP4WithHandRobotModule::bounds() const
  {
    return _bounds;
  }

  const std::map<std::string, std::vector<double> > & HRP4WithHandRobotModule::stance() const
  {
    return _stance;
  }

  HRP4VREPRobotModule::HRP4VREPRobotModule()
  {
    readUrdf("hrp4_vrep", filteredLinks);

    assert(_forceSensors[0].name() == "RightFootForceSensor");
    assert(_forceSensors[1].name() == "LeftFootForceSensor");
    _forceSensors[0] = mc_rbdyn::ForceSensor("RightFootForceSensor", "r_ankle", sva::PTransformd(Eigen::Vector3d(0., 0., 0.)));
    _forceSensors[1] = mc_rbdyn::ForceSensor("LeftFootForceSensor", "l_ankle", sva::PTransformd(Eigen::Vector3d(0., 0., 0.)));

    _springs.springsBodies = { "l_ankle", "r_ankle" }; //TODO: check these are the correct bodies
    auto fileByBodyName = stdCollisionsFiles(mb);
    _convexHull = getConvexHull(fileByBodyName);
    _bounds = nominalBounds(limits);
    _stance = halfSittingPose(mb);
  }
}
