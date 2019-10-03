/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
/** This file holds all mc_rtc::Configuration based serialization operation for mc_rbdyn objects */

#include <mc_rtc/Configuration.h>

/* Easily serialized/deserialized types */
#include <mc_rbdyn/Base.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/Flexibility.h>
#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/PolygonInterpolator.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Springs.h>
#include <mc_rbdyn/polygon_utils.h>

#include <Tasks/QPTasks.h>

/* Serialized/deserialized from/to shared pointers */
#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>

/* Require a Robots instance to be deserialized */
#include <mc_rbdyn/Contact.h>
#include <mc_rtc/logging.h>

#include <fstream>

namespace mc_rtc
{

/* This temporary macro makes the file much easier to skim through */
#define DECLARE_IO(...)                                     \
  template<>                                                \
  struct MC_RBDYN_DLLAPI ConfigurationLoader<__VA_ARGS__>   \
  {                                                         \
    static __VA_ARGS__ load(const mc_rtc::Configuration &); \
    static mc_rtc::Configuration save(const __VA_ARGS__ &); \
  };

DECLARE_IO(Eigen::Matrix<double, 6, Eigen::Dynamic>)

DECLARE_IO(sva::RBInertiad)

DECLARE_IO(rbd::Joint::Type)
DECLARE_IO(rbd::Joint)
DECLARE_IO(rbd::Body)
DECLARE_IO(rbd::MultiBody)
DECLARE_IO(rbd::MultiBodyConfig)

DECLARE_IO(tasks::qp::JointGains)

DECLARE_IO(rbd::parsers::Geometry::Box)
DECLARE_IO(rbd::parsers::Geometry::Cylinder)
DECLARE_IO(rbd::parsers::Geometry::Sphere)
DECLARE_IO(rbd::parsers::Geometry::Mesh)
DECLARE_IO(rbd::parsers::Geometry::Superellipsoid)
DECLARE_IO(rbd::parsers::Geometry)
DECLARE_IO(rbd::parsers::Visual)

DECLARE_IO(mc_rbdyn::Base)
DECLARE_IO(mc_rbdyn::BodySensor)
DECLARE_IO(mc_rbdyn::Collision)
DECLARE_IO(std::shared_ptr<mc_rbdyn::Surface>)
DECLARE_IO(std::shared_ptr<mc_rbdyn::PlanarSurface>)
DECLARE_IO(std::shared_ptr<mc_rbdyn::CylindricalSurface>)
DECLARE_IO(std::shared_ptr<mc_rbdyn::GripperSurface>)
DECLARE_IO(mc_rbdyn::Flexibility)
DECLARE_IO(mc_rbdyn::ForceSensor)
DECLARE_IO(mc_rbdyn::Plane)
DECLARE_IO(mc_rbdyn::PolygonInterpolator)
DECLARE_IO(mc_rbdyn::Springs)
DECLARE_IO(mc_rbdyn::Mimic)
DECLARE_IO(mc_rbdyn::RobotModule::Gripper)
DECLARE_IO(mc_rbdyn::RobotModule::Gripper::Safety)

#undef DECLARE_IO

template<>
struct MC_RBDYN_DLLAPI ConfigurationLoader<mc_rbdyn::RobotModule>
{
  static mc_rbdyn::RobotModule load(const mc_rtc::Configuration &);
  static mc_rtc::Configuration save(const mc_rbdyn::RobotModule &,
                                    bool save_mbc = true,
                                    const std::vector<std::string> & filteredLinks = {},
                                    bool fixed = false);
};

template<>
struct MC_RBDYN_DLLAPI ConfigurationLoader<mc_rbdyn::RobotModulePtr>
{
  static mc_rbdyn::RobotModulePtr load(const mc_rtc::Configuration &);
  static mc_rtc::Configuration save(const mc_rbdyn::RobotModulePtr &,
                                    bool save_mbc = true,
                                    const std::vector<std::string> & filteredLinks = {},
                                    bool fixed = false);
};

template<>
struct MC_RBDYN_DLLAPI ConfigurationLoader<mc_rbdyn::Contact>
{
  static mc_rbdyn::Contact load(const mc_rtc::Configuration &, const mc_rbdyn::Robots &);
  static mc_rtc::Configuration save(const mc_rbdyn::Contact &);
};

} // namespace mc_rtc
