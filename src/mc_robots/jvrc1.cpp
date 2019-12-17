/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef JVRC_DESCRIPTION_PATH
#  error "JVRC_DESCRIPTION_PATH must be defined to build this RobotModule"
#endif

#define JVRC_VAL(x) #x
#define JVRC_VAL_VAL(x) JVRC_VAL(x)

#include "jvrc1.h"

#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>

#include <fstream>
namespace bfs = boost::filesystem;

namespace mc_robots
{

JVRC1RobotModule::JVRC1RobotModule() : RobotModule(std::string(JVRC_VAL_VAL(JVRC_DESCRIPTION_PATH)), "jvrc1")
{
  std::ifstream ifs(urdf_path);
  if(ifs.is_open())
  {
    std::stringstream urdf;
    urdf << ifs.rdbuf();
    mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), false);
    mb = res.mb;
    mbc = res.mbc;
    mbg = res.mbg;
    boundsFromURDF(res.limits);
    _collisionTransforms = res.collision_tf;
    _visual = res.visual;

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
    expand_stance();
    make_default_ref_joint_order();
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

    auto lipmConf = mc_rtc::Configuration::fromYAMLData(
        "leftFootSurface: LeftFootCenter                                                                         \n"
        "rightFootSurface: RightFootCenter                                                                       \n"
        "torsoBodyName: WAIST_R_S                                                                                \n"
        "tasks:                                                                                                  \n"
        "  com:                                                                                                  \n"
        "    stiffness: [1000, 1000, 100]                                                                        \n"
        "    weight: 1000                                                                                        \n"
        "    active_joints: [Root,                                                                                 "
        "                    R_HIP_Y, R_HIP_R, R_HIP_P, R_KNEE, R_ANKLE_P, R_ANKLE_R,                              "
        "                    L_HIP_Y, L_HIP_R, L_HIP_P, L_KNEE, L_ANKLE_P, L_ANKLE_R]                            \n"
        "    height: 0.85                                                                                        \n"
        "    max_height: 0.90                                                                                    \n"
        "    min_height: 0.65                                                                                    \n"
        "  contact:                                                                                              \n"
        "    damping: 300                                                                                        \n"
        "    stiffness: 1                                                                                        \n"
        "    weight: 10000                                                                                       \n"
        "  pelvis:                                                                                               \n"
        "    stiffness: 10                                                                                       \n"
        "    weight: 100                                                                                         \n"
        "  torso:                                                                                                \n"
        "    stiffness: 10                                                                                       \n"
        "    weight: 100                                                                                         \n"
        "    pitch: 0                                                                                            \n"
        "    minPitch: -0.1                                                                                      \n"
        "    maxPitch: 0.4                                                                                       \n"
        "fdqp_weights:                                                                                           \n"
        "  net_wrench: 10000                                                                                     \n"
        "  ankle_torque: 100                                                                                     \n"
        "  pressure: 1                                                                                           \n"
        "vdc:                                                                                                    \n"
        "  frequency: 1                                                                                          \n"
        "  stiffness: 1000                                                                                       \n"
        "zmpcc:                                                                                                  \n"
        "  integrator_leak_rate: 0.1                                                                             \n"
        "admittance:                                                                                             \n"
        "  com: [0, 0]                                                                                           \n"
        "  cop: [0.01, 0.01]                                                                                     \n"
        "  dfz: 0.0001                                                                                           \n"
        "  dfz_damping: 0                                                                                        \n"
        "dcm_tracking:                                                                                           \n"
        "  gains:                                                                                                \n"
        "    prop: 5.0                                                                                           \n"
        "    integral: 10                                                                                        \n"
        "    deriv: 0.5                                                                                          \n"
        "  derivator_time_constant: 1                                                                            \n"
        "  integrator_time_constant: 10                                                                          \n"
        "sole:                                                                                                   \n"
        "  half_length: 0.11                                                                                     \n"
        "  half_width: 0.05                                                                                      \n"
        "  friction: 0.7                                                                                         ");
    _lipmStabilizerConfig.load(lipmConf);
  }
  else
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Could not load JVRC-1 model at " << urdf_path)
  }
}

} // namespace mc_robots
