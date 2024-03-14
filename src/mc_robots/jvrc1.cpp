/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef JVRC_DESCRIPTION_PATH
#  error "JVRC_DESCRIPTION_PATH must be defined to build this RobotModule"
#endif

#define JVRC_VAL(x) #x
#define JVRC_VAL_VAL(x) JVRC_VAL(x)

#include "jvrc1.h"

#include <mc_rbdyn/Device.h>
#include <mc_rbdyn/RobotModuleMacros.h>

#include <mc_rtc/logging.h>

#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>

#include <fstream>
namespace bfs = boost::filesystem;

namespace mc_robots
{

mc_rbdyn::DevicePtr JVRC1DummySpeaker::clone() const
{
  auto dummy = new JVRC1DummySpeaker(name_);
  dummy->text_ = text_;
  return mc_rbdyn::DevicePtr(dummy);
}

JVRC1RobotModule::JVRC1RobotModule(bool fixed, bool filter_mimics)
: RobotModule(std::string(JVRC_VAL_VAL(JVRC_DESCRIPTION_PATH)), "jvrc1")
{
  _canonicalParameters = {"JVRC1"};

  std::vector<std::string> filter_links = {};
  if(filter_mimics)
  {
    // clang-format off
    filter_links = {
      "R_UTHUMB_S",
      "R_LTHUMB_S",
      "R_UINDEX_S",
      "R_LINDEX_S",
      "R_ULITTLE_S",
      "R_LLITTLE_S",
      "L_UTHUMB_S",
      "L_LTHUMB_S",
      "L_UINDEX_S",
      "L_LINDEX_S",
      "L_ULITTLE_S",
      "L_LLITTLE_S"
    };
    // clang-format on
  }
  init(rbd::parsers::from_urdf_file(
      urdf_path,
      rbd::parsers::ParserParameters{}.fixed(fixed).filtered_links(filter_links).remove_filtered_links(true)));
  _ref_joint_order = {"R_HIP_P",      "R_HIP_R",      "R_HIP_Y",      "R_KNEE",       "R_ANKLE_R", "R_ANKLE_P",
                      "L_HIP_P",      "L_HIP_R",      "L_HIP_Y",      "L_KNEE",       "L_ANKLE_R", "L_ANKLE_P",
                      "WAIST_Y",      "WAIST_P",      "WAIST_R",      "NECK_Y",       "NECK_R",    "NECK_P",
                      "R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",    "R_ELBOW_Y", "R_WRIST_R",
                      "R_WRIST_Y",    "R_UTHUMB",     "R_LTHUMB",     "R_UINDEX",     "R_LINDEX",  "R_ULITTLE",
                      "R_LLITTLE",    "L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_P", "L_ELBOW_Y",
                      "L_WRIST_R",    "L_WRIST_Y",    "L_UTHUMB",     "L_LTHUMB",     "L_UINDEX",  "L_LINDEX",
                      "L_ULITTLE",    "L_LLITTLE"};
  _devices.emplace_back(new JVRC1DummySpeaker{"DummySpeaker"});

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
  for(size_t i = 0; i < rjo.size(); ++i) { _stance[rjo[i]] = {default_q[i]}; }
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.8275}};
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RightFootForceSensor", "R_ANKLE_P_S", sva::PTransformd::Identity()));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftFootForceSensor", "L_ANKLE_P_S", sva::PTransformd::Identity()));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("RightHandForceSensor", "R_WRIST_Y_S", sva::PTransformd::Identity()));
  _forceSensors.push_back(mc_rbdyn::ForceSensor("LeftHandForceSensor", "L_WRIST_Y_S", sva::PTransformd::Identity()));

  _bodySensors.emplace_back("Accelerometer", "PELVIS_S", sva::PTransformd(Eigen::Vector3d(-0.0325, 0, 0.1095)));
  _bodySensors.emplace_back("FloatingBase", "PELVIS_S", sva::PTransformd::Identity());

  for(size_t i = 0; i < rjo.size(); ++i) { _jointSensors.push_back(mc_rbdyn::JointSensor(rjo[i])); }

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

#ifndef MC_RTC_BUILD_STATIC

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"JVRC1", "JVRC1Fixed", "JVRC1NoHands", "JVRC1NoHandsFixed"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name)
  {
    if(name == "JVRC1") { return new mc_robots::JVRC1RobotModule(false); }
    else if(name == "JVRC1Fixed") { return new mc_robots::JVRC1RobotModule(true); }
    if(name == "JVRC1NoHands") { return new mc_robots::JVRC1RobotModule(false, true); }
    else if(name == "JVRC1NoHandsFixed") { return new mc_robots::JVRC1RobotModule(true, true); }
    else
    {
      mc_rtc::log::error("JVRC1 module Cannot create an object of type {}", name);
      return nullptr;
    }
  }
}

#else

#  include <mc_rbdyn/RobotLoader.h>

namespace
{

static auto registered = []()
{
  using fn_t = std::function<mc_robots::JVRC1RobotModule *()>;
  mc_rbdyn::RobotLoader::register_object("JVRC1", fn_t([]() { return new mc_robots::JVRC1RobotModule(false); }));
  mc_rbdyn::RobotLoader::register_object("JVRC1NoHands",
                                         fn_t([]() { return new mc_robots::JVRC1RobotModule(false, true); }));
  mc_rbdyn::RobotLoader::register_object("JVRC1NoHandsFixed",
                                         fn_t([]() { return new mc_robots::JVRC1RobotModule(true, true); }));
  mc_rbdyn::RobotLoader::register_object("JVRC1Fixed", fn_t([]() { return new mc_robots::JVRC1RobotModule(true); }));
  return true;
}();
} // namespace

#endif
