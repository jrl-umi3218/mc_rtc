/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef JVRC_DESCRIPTION_PATH
#  error "JVRC_DESCRIPTION_PATH must be defined to build this RobotModule"
#endif

#define JVRC_VAL(x) #x
#define JVRC_VAL_VAL(x) JVRC_VAL(x)

#include "jvrc1.h"

#include <mc_rbdyn/RobotModuleMacros.h>

#include <mc_rtc/logging.h>

#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>

#include <fstream>
namespace bfs = boost::filesystem;

namespace mc_robots
{

JVRC1RobotModule::JVRC1RobotModule() : RobotModule(std::string(JVRC_VAL_VAL(JVRC_DESCRIPTION_PATH)), "jvrc1")
{
  init(rbd::parsers::from_urdf_file(urdf_path, false));

  std::string convexPath = path + "/convex/" + name + "/";
  bfs::path p(convexPath);
  if(bfs::exists(p) && bfs::is_directory(p))
  {
    std::vector<bfs::path> files;
    std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
    for(const bfs::path & file : files)
    {
      size_t off = file.filename().string().rfind("-ch.txt");
      if(off != std::string::npos)
      {
        std::string name = file.filename().string();
        name.replace(off, 7, "");
        _convexHull[name] = std::pair<std::string, std::string>(name, file.string());
      }
    }
  }
  std::vector<double> default_q = {-0.38, -0.01, 0., 0.72, -0.01, -0.33,  -0.38, 0.02, 0.,    0.72, -0.02, -0.33, 0.,
                                   0.13,  0.,    0., 0.,   0.,    -0.052, -0.17, 0.,   -0.52, 0.,   0.,    0.,    0.,
                                   0.,    0.,    0., 0.,   0.,    -0.052, 0.17,  0.,   -0.52, 0.,   0.,    0.,    0.,
                                   0.,    0.,    0., 0.,   0.,    0.,     0.,    0.,   0.,    0.,   0.};
  const auto & rjo = ref_joint_order();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    _stance[rjo[i]] = {default_q[i]};
  }
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.8275}};
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RightFootForceSensor", "R_ANKLE_P_S", sva::PTransformd::Identity()));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftFootForceSensor", "L_ANKLE_P_S", sva::PTransformd::Identity()));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RightHandForceSensor", "R_WRIST_Y_S", sva::PTransformd::Identity()));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "L_WRIST_Y_S", sva::PTransformd::Identity()));

  _bodySensors.emplace_back("Accelerometer", "PELVIS_S", sva::PTransformd(Eigen::Vector3d(-0.0325, 0, 0.1095)));
  _bodySensors.emplace_back("FloatingBase", "PELVIS_S", sva::PTransformd::Identity());

  _minimalSelfCollisions = {
      {"WAIST_R_S", "L_SHOULDER_Y_S", 0.02, 0.001, 0.}, {"WAIST_R_S", "R_SHOULDER_Y_S", 0.02, 0.001, 0.},
      {"PELVIS_S", "R_ELBOW_P_S", 0.05, 0.001, 0.},     {"PELVIS_S", "L_ELBOW_P_S", 0.05, 0.001, 0.},
      {"R_WRIST_Y_S", "R_HIP_Y_S", 0.05, 0.025, 0.},    {"L_WRIST_Y_S", "L_HIP_Y_S", 0.05, 0.025, 0.}};
  _commonSelfCollisions = _minimalSelfCollisions;
  _grippers = {{"l_gripper", {"L_UTHUMB"}, true}, {"r_gripper", {"R_UTHUMB"}, false}};

  // Configure the stabilizer. Uses the default values of the
  // StabilizerConfiguration except for where values specific to the JVRC
  // robot are required.
  _lipmStabilizerConfig.leftFootSurface = "LeftFootCenter";
  _lipmStabilizerConfig.rightFootSurface = "RightFootCenter";
  _lipmStabilizerConfig.torsoBodyName = "WAIST_R_S";
  _lipmStabilizerConfig.comHeight = 0.85;
  _lipmStabilizerConfig.comActiveJoints = {"Root",      "R_HIP_Y",   "R_HIP_R",  "R_HIP_P", "R_KNEE",
                                           "R_ANKLE_P", "R_ANKLE_R", "L_HIP_Y",  "L_HIP_R", "L_HIP_P",
                                           "L_KNEE",    "L_ANKLE_P", "L_ANKLE_R"};
  _lipmStabilizerConfig.torsoPitch = 0;
  _lipmStabilizerConfig.copAdmittance = Eigen::Vector2d{0.01, 0.01};
  _lipmStabilizerConfig.dcmPropGain = 5.0;
  _lipmStabilizerConfig.dcmIntegralGain = 10;
  _lipmStabilizerConfig.dcmDerivGain = 0.5;
  _lipmStabilizerConfig.dcmDerivatorTimeConstant = 1;
  _lipmStabilizerConfig.dcmIntegratorTimeConstant = 10;
}

} // namespace mc_robots

ROBOT_MODULE_DEFAULT_CONSTRUCTOR("JVRC1", mc_robots::JVRC1RobotModule);
