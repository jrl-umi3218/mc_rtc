/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/configuration_io.h>

#include <RBDyn/parsers/urdf.h>

#include <sch-core/STP_BV.h>
#include <sch-core/STP_BV_P.h>
#include <sch-core/S_Box.h>
#include <sch-core/S_Capsule.h>
#include <sch-core/S_Cone.h>
#include <sch-core/S_Cylinder.h>
#include <sch-core/S_Point.h>
#include <sch-core/S_Polyhedron.h>
#include <sch-core/S_Sphere.h>
#include <sch-core/S_Superellipsoid.h>

#include <mc_rtc/deprecated.h>
#include <filesystem>
namespace fs = std::filesystem;

namespace fmt
{

template<>
struct formatter<rbd::Joint::Type> : public formatter<string_view>
{
  template<typename FormatContext>
  auto format(const rbd::Joint::Type & t, FormatContext & ctx) -> decltype(ctx.out())
  {
    switch(t)
    {
      case rbd::Joint::Type::Rev:
        return formatter<string_view>::format("rev", ctx);
      case rbd::Joint::Type::Prism:
        return formatter<string_view>::format("prism", ctx);
      case rbd::Joint::Type::Spherical:
        return formatter<string_view>::format("spherical", ctx);
      case rbd::Joint::Type::Planar:
        return formatter<string_view>::format("planar", ctx);
      case rbd::Joint::Type::Cylindrical:
        return formatter<string_view>::format("cylindrical", ctx);
      case rbd::Joint::Type::Free:
        return formatter<string_view>::format("free", ctx);
      case rbd::Joint::Type::Fixed:
        return formatter<string_view>::format("fixed", ctx);
      default:
        return formatter<string_view>::format(fmt::to_string(static_cast<std::underlying_type_t<rbd::Joint::Type>>(t)),
                                              ctx);
    }
  }
};

} // namespace fmt

namespace
{
// Return relative path to go to "to" from "from"
fs::path relative(fs::path to, fs::path from)
{
  fs::path::const_iterator fromIter = from.begin();
  fs::path::const_iterator toIter = to.begin();
  while(fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter))
  {
    ++toIter;
    ++fromIter;
  }
  fs::path relPath;
  while(fromIter != from.end() && *fromIter != fs::path("."))
  {
    relPath /= "..";
    ++fromIter;
  }
  while(toIter != to.end())
  {
    relPath /= *toIter;
    ++toIter;
  }
  return relPath;
}
} // namespace

namespace mc_rtc
{

rbd::Joint::Type ConfigurationLoader<rbd::Joint::Type>::load(const mc_rtc::Configuration & config)
{
  std::string type = "";
  config("type", type);
  if(type == "rev") { return rbd::Joint::Type::Rev; }
  if(type == "prism") { return rbd::Joint::Type::Prism; }
  if(type == "spherical") { return rbd::Joint::Type::Spherical; }
  if(type == "planar") { return rbd::Joint::Type::Planar; }
  if(type == "cylindrical") { return rbd::Joint::Type::Cylindrical; }
  if(type == "free") { return rbd::Joint::Type::Free; }
  if(type == "fixed") { return rbd::Joint::Type::Fixed; }
  mc_rtc::log::error_and_throw("{} was stored as joint type, cannot comprehend that", type);
}

mc_rtc::Configuration ConfigurationLoader<rbd::Joint::Type>::save(const rbd::Joint::Type & type)
{
  mc_rtc::Configuration config;
  config.add("type", fmt::to_string(type));
  return config;
}

mc_rbdyn::Base ConfigurationLoader<mc_rbdyn::Base>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("X_0_s"), config("X_b0_s"), config("type")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Base>::save(const mc_rbdyn::Base & b)
{
  mc_rtc::Configuration config;
  config.add("name", b.baseName);
  config.add("X_0_s", b.X_0_s);
  config.add("X_b0_s", b.X_b0_s);
  config.add("type", b.baseType);
  return config;
}

mc_rbdyn::BodySensor ConfigurationLoader<mc_rbdyn::BodySensor>::load(const mc_rtc::Configuration & config)
{
  return mc_rbdyn::BodySensor(config("name"), config("parentBody"), config("X_b_s"));
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::BodySensor>::save(const mc_rbdyn::BodySensor & bs)
{
  mc_rtc::Configuration config;
  config.add("name", bs.name());
  config.add("parentBody", bs.parentBody());
  config.add("X_b_s", bs.X_b_s());
  return config;
}

mc_rbdyn::JointSensor ConfigurationLoader<mc_rbdyn::JointSensor>::load(const mc_rtc::Configuration & config)
{
  return mc_rbdyn::JointSensor(static_cast<std::string>(config("joint")));
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::JointSensor>::save(const mc_rbdyn::JointSensor & js)
{
  mc_rtc::Configuration config;
  config.add("joint", js.joint());
  return config;
}

mc_rbdyn::Collision ConfigurationLoader<mc_rbdyn::Collision>::load(const mc_rtc::Configuration & config)
{
  auto body1 = config("body1");
  auto body2 = config("body2");
  auto loadActiveJoints = [&](std::string prefix) -> std::tuple<std::optional<std::vector<std::string>>, bool>
  {
    auto active_joints = config.find(prefix + "ActiveJoints");
    auto joints = config.find(prefix + "Joints");
    auto inactive_joints = config.find(prefix + "InactiveJoints");
    if((active_joints || joints) && inactive_joints)
    {
      mc_rtc::log::warning(
          "Collision ({} - {}) has both {}ActiveJoints and {}InactiveJoints, {}ActiveJoints will be used", body1, body2,
          prefix);
    }

    if(joints)
    {
      mc_rtc::log::deprecated(fmt::format("Collision ({} - {})", body1, body2), prefix + "Joints",
                              prefix + "ActiveJoints");
      auto jointsV = joints->operator std::vector<std::string>();
      if(jointsV.empty())
      {
        mc_rtc::log::warning(
            "[Collision][breaking change] The meaning of an empty joint vector has changed from all joints active to "
            "no joints active. Remove {}Joints from your configuration to restore the former behaviour.",
            prefix);
      }
      return {jointsV, false};
    }
    if(active_joints) { return {active_joints->operator std::vector<std::string>(), false}; }
    if(inactive_joints) { return {inactive_joints->operator std::vector<std::string>(), true}; }
    return {std::nullopt, false};
  };
  const auto & [r1Joints, r1Inactive] = loadActiveJoints("r1");
  const auto & [r2Joints, r2Inactive] = loadActiveJoints("r2");
  return mc_rbdyn::Collision(body1, body2, config("iDist", 0.05), config("sDist", 0.01), config("damping", 0.0),
                             r1Joints, r2Joints, r1Inactive, r2Inactive);
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Collision>::save(const mc_rbdyn::Collision & c)
{
  mc_rtc::Configuration config;
  config.add("body1", c.body1);
  config.add("body2", c.body2);
  config.add("iDist", c.iDist);
  config.add("sDist", c.sDist);
  config.add("damping", c.damping);
  auto saveActiveJoints = [&](std::string prefix, const std::optional<std::vector<std::string>> & joints, bool inactive)
  {
    if(joints)
    {
      if(inactive) { config.add(prefix + "InactiveJoints", *c.r1Joints); }
      else { config.add(prefix + "ActiveJoints", *c.r1Joints); }
    }
  };
  saveActiveJoints("r1", c.r1Joints, c.r1JointsInactive);
  saveActiveJoints("r2", c.r2Joints, c.r2JointsInactive);
  return config;
}

std::shared_ptr<mc_rbdyn::Surface> ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::load(
    const mc_rtc::Configuration & config)
{
  std::string type = config("type");
  if(type == "planar")
  {
    return std::make_shared<mc_rbdyn::PlanarSurface>(config("name"), config("bodyName"), config("X_b_s"),
                                                     config("materialName"), config("planarPoints"));
  }
  else if(type == "cylindrical")
  {
    return std::make_shared<mc_rbdyn::CylindricalSurface>(config("name"), config("bodyName"), config("X_b_s"),
                                                          config("materialName"), config("radius"), config("width"));
  }
  else if(type == "gripper")
  {
    return std::make_shared<mc_rbdyn::GripperSurface>(config("name"), config("bodyName"), config("X_b_s"),
                                                      config("materialName"), config("pointsFromOrigin"),
                                                      config("X_b_motor"), config("motorMaxTorque"));
  }
  mc_rtc::log::error_and_throw("Unknown surface type stored {}", type);
}

mc_rtc::Configuration ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::save(
    const std::shared_ptr<mc_rbdyn::Surface> & s)
{
  mc_rtc::Configuration config;
  config.add("type", s->type());
  config.add("name", s->name());
  config.add("bodyName", s->bodyName());
  config.add("X_b_s", s->X_b_s());
  config.add("materialName", s->materialName());
  if(s->type() == "planar")
  {
    auto ps = static_cast<mc_rbdyn::PlanarSurface *>(s.get());
    config.add("planarPoints", ps->planarPoints());
  }
  else if(s->type() == "cylindrical")
  {
    auto cs = static_cast<mc_rbdyn::CylindricalSurface *>(s.get());
    config.add("radius", cs->radius());
    config.add("width", cs->width());
  }
  else if(s->type() == "gripper")
  {
    auto gs = static_cast<mc_rbdyn::GripperSurface *>(s.get());
    config.add("pointsFromOrigin", gs->pointsFromOrigin());
    config.add("X_b_motor", gs->X_b_motor());
    config.add("motorMaxTorque", gs->motorMaxTorque());
  }
  else { mc_rtc::log::error_and_throw("Cannot serialize a surface of type {}", s->type()); }
  return config;
}

std::shared_ptr<mc_rbdyn::PlanarSurface> ConfigurationLoader<std::shared_ptr<mc_rbdyn::PlanarSurface>>::load(
    const mc_rtc::Configuration & config)
{
  std::string type = config("type");
  if(type != "planar")
  {
    mc_rtc::log::error_and_throw("Tried to deserialize a non-planar surface into a planar surface");
  }
  return std::static_pointer_cast<mc_rbdyn::PlanarSurface>(
      ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::load(config));
}

mc_rtc::Configuration ConfigurationLoader<std::shared_ptr<mc_rbdyn::PlanarSurface>>::save(
    const std::shared_ptr<mc_rbdyn::PlanarSurface> & s)
{
  return ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::save(s);
}

std::shared_ptr<mc_rbdyn::CylindricalSurface> ConfigurationLoader<std::shared_ptr<mc_rbdyn::CylindricalSurface>>::load(
    const mc_rtc::Configuration & config)
{
  std::string type = config("type");
  if(type != "cylindrical")
  {
    mc_rtc::log::error_and_throw("Tried to deserialize a non-cylindrical surface into a cylindrical surface");
  }
  return std::static_pointer_cast<mc_rbdyn::CylindricalSurface>(
      ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::load(config));
}

mc_rtc::Configuration ConfigurationLoader<std::shared_ptr<mc_rbdyn::CylindricalSurface>>::save(
    const std::shared_ptr<mc_rbdyn::CylindricalSurface> & s)
{
  return ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::save(s);
}

std::shared_ptr<mc_rbdyn::GripperSurface> ConfigurationLoader<std::shared_ptr<mc_rbdyn::GripperSurface>>::load(
    const mc_rtc::Configuration & config)
{
  std::string type = config("type");
  if(type != "gripper")
  {
    mc_rtc::log::error_and_throw("Tried to deserialize a non-gripper surface into a gripper surface");
  }
  return std::static_pointer_cast<mc_rbdyn::GripperSurface>(
      ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::load(config));
}

mc_rtc::Configuration ConfigurationLoader<std::shared_ptr<mc_rbdyn::GripperSurface>>::save(
    const std::shared_ptr<mc_rbdyn::GripperSurface> & s)
{
  return ConfigurationLoader<std::shared_ptr<mc_rbdyn::Surface>>::save(s);
}

mc_rbdyn::Flexibility ConfigurationLoader<mc_rbdyn::Flexibility>::load(const mc_rtc::Configuration & config)
{
  return {config("jointName"), config("K"), config("C"), config("O")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Flexibility>::save(const mc_rbdyn::Flexibility & flex)
{
  mc_rtc::Configuration config;
  config.add("jointName", flex.jointName);
  config.add("K", flex.K);
  config.add("C", flex.C);
  config.add("O", flex.O);
  return config;
}

mc_rbdyn::ForceSensor ConfigurationLoader<mc_rbdyn::ForceSensor>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("parentBody"), config("X_p_f")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::ForceSensor>::save(const mc_rbdyn::ForceSensor & fs)
{
  mc_rtc::Configuration config;
  config.add("name", fs.name());
  config.add("parentBody", fs.parentBody());
  config.add("X_p_f", fs.X_p_f());
  return config;
}

mc_rbdyn::Plane ConfigurationLoader<mc_rbdyn::Plane>::load(const mc_rtc::Configuration & config)
{
  return mc_rbdyn::Plane{config("normal"), config("offset")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Plane>::save(const mc_rbdyn::Plane & pl)
{
  mc_rtc::Configuration config;
  config.add("normal", pl.normal);
  config.add("offset", pl.offset);
  return config;
}

mc_rbdyn::PolygonInterpolator ConfigurationLoader<mc_rbdyn::PolygonInterpolator>::load(
    const mc_rtc::Configuration & config)
{
  std::vector<mc_rbdyn::PolygonInterpolator::tuple_pair_t> vec = config("tuple_pairs");
  return mc_rbdyn::PolygonInterpolator(vec);
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::PolygonInterpolator>::save(const mc_rbdyn::PolygonInterpolator & pi)
{
  mc_rtc::Configuration config;
  config.add("tuple_pairs", pi.tuple_pairs());
  return config;
}

mc_rbdyn::Springs ConfigurationLoader<mc_rbdyn::Springs>::load(const mc_rtc::Configuration & config)
{
  mc_rbdyn::Springs spr;
  spr.springsBodies = config("springsBodies");
  spr.afterSpringsBodies = config("afterSpringsBodies");
  spr.springsJoints = config("springsJoints");
  return spr;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Springs>::save(const mc_rbdyn::Springs & spr)
{
  mc_rtc::Configuration config;
  config.add("springsBodies", spr.springsBodies);
  config.add("afterSpringsBodies", spr.afterSpringsBodies);
  config.add("springsJoints", spr.springsJoints);
  return config;
}

sva::RBInertiad ConfigurationLoader<sva::RBInertiad>::load(const mc_rtc::Configuration & config)
{
  Eigen::Matrix3d inertia = config("inertia");
  return {config("mass"), config("momentum"), inertia};
}

mc_rtc::Configuration ConfigurationLoader<sva::RBInertiad>::save(const sva::RBInertiad & rbi)
{
  mc_rtc::Configuration config;
  config.add("mass", rbi.mass());
  config.add("momentum", rbi.momentum());
  config.add("inertia", rbi.inertia());
  return config;
}

rbd::Body ConfigurationLoader<rbd::Body>::load(const mc_rtc::Configuration & config)
{
  return {config("inertia"), config("name")};
}

mc_rtc::Configuration ConfigurationLoader<rbd::Body>::save(const rbd::Body & bod)
{
  mc_rtc::Configuration config;
  config.add("name", bod.name());
  config.add("inertia", bod.inertia());
  return config;
}

rbd::Joint ConfigurationLoader<rbd::Joint>::load(const mc_rtc::Configuration & config)
{
  rbd::Joint j{config("type"), config("axis"), config("forward"), config("name")};
  bool isMimic = config("isMimic");
  if(isMimic) { j.makeMimic(config("mimicName"), config("mimicMultiplier"), config("mimicOffset")); }
  return j;
}

mc_rtc::Configuration ConfigurationLoader<rbd::Joint>::save(const rbd::Joint & j)
{
  mc_rtc::Configuration config;
  config.add("type", j.type());
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
  switch(j.type())
  {
    case rbd::Joint::Rev:
    case rbd::Joint::Cylindrical:
      axis = j.direction() * j.motionSubspace().col(0).head<3>();
      break;
    case rbd::Joint::Prism:
      axis = j.direction() * j.motionSubspace().col(0).tail<3>();
      break;
    default:
      break;
  }
  config.add("axis", axis);
  config.add("forward", j.forward());
  config.add("isMimic", j.isMimic());
  if(j.isMimic())
  {
    config.add("mimicName", j.mimicName());
    config.add("mimicMultiplier", j.mimicMultiplier());
    config.add("mimicOffset", j.mimicOffset());
  }
  config.add("name", j.name());
  return config;
}

rbd::MultiBody ConfigurationLoader<rbd::MultiBody>::load(const mc_rtc::Configuration & config)
{
  return {config("bodies"), config("joints"),  config("preds"),
          config("succs"),  config("parents"), config("transforms")};
}

mc_rtc::Configuration ConfigurationLoader<rbd::MultiBody>::save(const rbd::MultiBody & mb)
{
  mc_rtc::Configuration config;
  config.add("bodies", mb.bodies());
  config.add("joints", mb.joints());
  config.add("preds", mb.predecessors());
  config.add("succs", mb.successors());
  config.add("parents", mb.parents());
  config.add("transforms", mb.transforms());
  return config;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> ConfigurationLoader<Eigen::Matrix<double, 6, Eigen::Dynamic>>::load(
    const mc_rtc::Configuration & config)
{
  Eigen::Matrix<double, 6, Eigen::Dynamic> m(6, static_cast<int>(config("cols")));
  auto data = config("data");
  if(static_cast<Eigen::DenseIndex>(data.size()) != 6 * m.cols())
  {
    auto msg = fmt::format("Stored data size ({}) is different from the expected size ({})", data.size(), 6 * m.cols());
    mc_rtc::log::critical(msg);
    throw mc_rtc::Configuration::Exception(msg, data);
  }
  for(Eigen::DenseIndex i = 0; i < 6; ++i)
  {
    for(Eigen::DenseIndex j = 0; j < m.cols(); ++j) { m(i, j) = data[static_cast<size_t>(m.cols() * i + j)]; }
  }
  return m;
}

mc_rtc::Configuration ConfigurationLoader<Eigen::Matrix<double, 6, Eigen::Dynamic>>::save(
    const Eigen::Matrix<double, 6, Eigen::Dynamic> & m)
{
  mc_rtc::Configuration config;
  config.add("cols", static_cast<int>(m.cols()));
  auto data = config.array("data", static_cast<size_t>(6 * m.cols()));
  for(Eigen::DenseIndex i = 0; i < 6; ++i)
  {
    for(Eigen::DenseIndex j = 0; j < m.cols(); ++j) { data.push(m(i, j)); }
  }
  return config;
}

rbd::MultiBodyConfig ConfigurationLoader<rbd::MultiBodyConfig>::load(const mc_rtc::Configuration & config)
{
  rbd::MultiBodyConfig mbc;
  mbc.q = config("q");
  mbc.alpha = config("alpha");
  mbc.alphaD = config("alphaD");
  mbc.force = config("force");
  mbc.jointConfig = config("jointConfig");
  mbc.jointVelocity = config("jointVelocity");
  mbc.jointTorque = config("jointTorque");
  mbc.motionSubspace = config("motionSubspace");
  mbc.bodyPosW = config("bodyPosW");
  mbc.parentToSon = config("parentToSon");
  mbc.bodyVelW = config("bodyVelW");
  mbc.bodyVelB = config("bodyVelB");
  mbc.bodyAccB = config("bodyAccB");
  mbc.gravity = config("gravity");
  return mbc;
}

mc_rtc::Configuration ConfigurationLoader<rbd::MultiBodyConfig>::save(const rbd::MultiBodyConfig & mbc)
{
  mc_rtc::Configuration config;
  config.add("q", mbc.q);
  config.add("alpha", mbc.alpha);
  config.add("alphaD", mbc.alphaD);
  config.add("force", mbc.force);
  config.add("jointConfig", mbc.jointConfig);
  config.add("jointVelocity", mbc.jointVelocity);
  config.add("jointTorque", mbc.jointTorque);
  config.add("motionSubspace", mbc.motionSubspace);
  config.add("bodyPosW", mbc.bodyPosW);
  config.add("parentToSon", mbc.parentToSon);
  config.add("bodyVelW", mbc.bodyVelW);
  config.add("bodyVelB", mbc.bodyVelB);
  config.add("bodyAccB", mbc.bodyAccB);
  config.add("gravity", mbc.gravity);
  return config;
}

mc_rbdyn::Mimic ConfigurationLoader<mc_rbdyn::Mimic>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("joint"), config("multiplier", 1.0), config("offset", 0.0)};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Mimic>::save(const mc_rbdyn::Mimic & mimic)
{
  mc_rtc::Configuration config;
  config.add("name", mimic.name);
  config.add("joint", mimic.joint);
  config.add("multiplier", mimic.multiplier);
  config.add("offset", mimic.offset);
  return config;
}

namespace
{

mc_rbdyn::RobotModule::Gripper loadGripper(const mc_rtc::Configuration & config,
                                           const mc_rbdyn::RobotModule::Gripper::Safety & safety)
{
  const std::string & name = config("name");
  const std::vector<std::string> & joints = config("joints");
  bool reverse_limits = config("reverse_limits");
  if(config.has("safety"))
  {
    const mc_rbdyn::RobotModule::Gripper::Safety & safety = config("safety");
    if(config.has("mimics")) { return {name, joints, reverse_limits, safety, config("mimics")}; }
    return {name, joints, reverse_limits, safety};
  }
  if(config.has("mimics")) { return {name, joints, reverse_limits, safety, config("mimics")}; }
  return {name, joints, reverse_limits};
}

} // namespace

mc_rbdyn::RobotModule::Gripper ConfigurationLoader<mc_rbdyn::RobotModule::Gripper>::load(
    const mc_rtc::Configuration & config)
{
  return loadGripper(config, {});
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModule::Gripper>::save(
    const mc_rbdyn::RobotModule::Gripper & rmg)
{
  mc_rtc::Configuration config;
  config.add("name", rmg.name);
  config.add("joints", rmg.joints);
  config.add("reverse_limits", rmg.reverse_limits);
  auto safety = rmg.safety();
  if(safety) { config.add("safety", *safety); }
  auto mimics = rmg.mimics();
  if(mimics) { config.add("mimics", *mimics); }
  return config;
}

mc_rbdyn::RobotModule::Gripper::Safety ConfigurationLoader<mc_rbdyn::RobotModule::Gripper::Safety>::load(
    const mc_rtc::Configuration & config)
{
  mc_rbdyn::RobotModule::Gripper::Safety safety;
  safety.load(config);
  return safety;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModule::Gripper::Safety>::save(
    const mc_rbdyn::RobotModule::Gripper::Safety & safety)
{
  return safety.save();
}

mc_rbdyn::RobotModule::FrameDescription ConfigurationLoader<mc_rbdyn::RobotModule::FrameDescription>::load(
    const mc_rtc::Configuration & config)
{
  return {config("name"), config("parent"), config("X_p_f"), config("baked", false)};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModule::FrameDescription>::save(
    const mc_rbdyn::RobotModule::FrameDescription & frame)
{
  mc_rtc::Configuration config;
  config.add("name", frame.name);
  config.add("parent", frame.parent);
  config.add("X_p_f", frame.X_p_f);
  config.add("baked", frame.baked);
  return config;
}

rbd::parsers::Geometry::Box ConfigurationLoader<rbd::parsers::Geometry::Box>::load(const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Box b;
  b.size = config("size");
  return b;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Box>::save(const rbd::parsers::Geometry::Box & b)
{
  mc_rtc::Configuration config;
  config.add("size", b.size);
  return config;
}

rbd::parsers::Geometry::Cylinder ConfigurationLoader<rbd::parsers::Geometry::Cylinder>::load(
    const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Cylinder c;
  c.radius = config("radius");
  c.length = config("length");
  return c;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Cylinder>::save(
    const rbd::parsers::Geometry::Cylinder & c)
{
  mc_rtc::Configuration config;
  config.add("radius", c.radius);
  config.add("length", c.length);
  return config;
}

rbd::parsers::Geometry::Sphere ConfigurationLoader<rbd::parsers::Geometry::Sphere>::load(
    const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Sphere s;
  s.radius = config("radius");
  return s;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Sphere>::save(const rbd::parsers::Geometry::Sphere & s)
{
  mc_rtc::Configuration config;
  config.add("radius", s.radius);
  return config;
}

rbd::parsers::Geometry::Mesh ConfigurationLoader<rbd::parsers::Geometry::Mesh>::load(
    const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Mesh m;
  m.filename = static_cast<std::string>(config("filename"));
  m.scaleV = config("scaleV");
  return m;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Mesh>::save(const rbd::parsers::Geometry::Mesh & m)
{
  mc_rtc::Configuration config;
  config.add("filename", m.filename);
  config.add("scaleV", m.scaleV);
  return config;
}

rbd::parsers::Geometry::Superellipsoid ConfigurationLoader<rbd::parsers::Geometry::Superellipsoid>::load(
    const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry::Superellipsoid s;
  s.size = config("size");
  s.epsilon1 = config("epsilon1");
  s.epsilon2 = config("epsilon2");
  return s;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry::Superellipsoid>::save(
    const rbd::parsers::Geometry::Superellipsoid & s)
{
  mc_rtc::Configuration config;
  config.add("size", s.size);
  config.add("epsilon1", s.epsilon1);
  config.add("epsilon2", s.epsilon2);
  return config;
}

rbd::parsers::Geometry ConfigurationLoader<rbd::parsers::Geometry>::load(const mc_rtc::Configuration & config)
{
  rbd::parsers::Geometry geom;
  if(config.has("box"))
  {
    geom.type = rbd::parsers::Geometry::Type::BOX;
    rbd::parsers::Geometry::Box b = config("box");
    geom.data = b;
  }
  else if(config.has("cylinder"))
  {
    geom.type = rbd::parsers::Geometry::Type::CYLINDER;
    rbd::parsers::Geometry::Cylinder b = config("cylinder");
    geom.data = b;
  }
  else if(config.has("sphere"))
  {
    geom.type = rbd::parsers::Geometry::Type::SPHERE;
    rbd::parsers::Geometry::Sphere b = config("sphere");
    geom.data = b;
  }
  else if(config.has("mesh"))
  {
    geom.type = rbd::parsers::Geometry::Type::MESH;
    rbd::parsers::Geometry::Mesh b = config("mesh");
    geom.data = b;
  }
  else if(config.has("superellipsoid"))
  {
    geom.type = rbd::parsers::Geometry::Type::SUPERELLIPSOID;
    rbd::parsers::Geometry::Superellipsoid s = config("superellipsoid");
    geom.data = s;
  }
  return geom;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Geometry>::save(const rbd::parsers::Geometry & geom)
{
  mc_rtc::Configuration config;
  switch(geom.type)
  {
    case rbd::parsers::Geometry::Type::BOX:
      config.add("box", boost::get<rbd::parsers::Geometry::Box>(geom.data));
      break;
    case rbd::parsers::Geometry::Type::CYLINDER:
      config.add("cylinder", boost::get<rbd::parsers::Geometry::Cylinder>(geom.data));
      break;
    case rbd::parsers::Geometry::Type::SPHERE:
      config.add("sphere", boost::get<rbd::parsers::Geometry::Sphere>(geom.data));
      break;
    case rbd::parsers::Geometry::Type::MESH:
      config.add("mesh", boost::get<rbd::parsers::Geometry::Mesh>(geom.data));
      break;
    case rbd::parsers::Geometry::Type::SUPERELLIPSOID:
      config.add("superellipsoid", boost::get<rbd::parsers::Geometry::Superellipsoid>(geom.data));
      break;
    default:
      break;
  }
  return config;
}

rbd::parsers::Material::Color ConfigurationLoader<rbd::parsers::Material::Color>::load(
    const mc_rtc::Configuration & config)
{
  return rbd::parsers::Material::Color{config("r"), config("g"), config("b"), config("a")};
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Material::Color>::save(const rbd::parsers::Material::Color & col)
{
  mc_rtc::Configuration config;
  config.add("r", col.r);
  config.add("g", col.g);
  config.add("b", col.b);
  config.add("a", col.a);
  return config;
}

rbd::parsers::Material::Texture ConfigurationLoader<rbd::parsers::Material::Texture>::load(
    const mc_rtc::Configuration & config)
{
  return rbd::parsers::Material::Texture{config("filename")};
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Material::Texture>::save(
    const rbd::parsers::Material::Texture & text)
{
  mc_rtc::Configuration config;
  config.add("filename", text.filename);
  return config;
}

rbd::parsers::Material ConfigurationLoader<rbd::parsers::Material>::load(const mc_rtc::Configuration & config)
{
  rbd::parsers::Material mat;
  if(config.has("color"))
  {
    mat.type = rbd::parsers::Material::Type::COLOR;
    rbd::parsers::Material::Color c = config("color");
    mat.data = c;
  }
  else if(config.has("texture"))
  {
    mat.type = rbd::parsers::Material::Type::TEXTURE;
    rbd::parsers::Material::Texture t = config("texture");
    mat.data = t;
  }
  return mat;
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Material>::save(const rbd::parsers::Material & mat)
{
  mc_rtc::Configuration config;
  switch(mat.type)
  {
    case rbd::parsers::Material::Type::COLOR:
      config.add("color", boost::get<rbd::parsers::Material::Color>(mat.data));
      break;
    case rbd::parsers::Material::Type::TEXTURE:
      config.add("texture", boost::get<rbd::parsers::Material::Texture>(mat.data));
      break;
    default:
      break;
  }
  return config;
}

rbd::parsers::Visual ConfigurationLoader<rbd::parsers::Visual>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("origin"), config("geometry"), config("material")};
}

mc_rtc::Configuration ConfigurationLoader<rbd::parsers::Visual>::save(const rbd::parsers::Visual & vis)
{
  mc_rtc::Configuration config;
  config.add("name", vis.name);
  config.add("origin", vis.origin);
  config.add("geometry", vis.geometry);
  config.add("material", vis.material);
  return config;
}

mc_rbdyn::S_ObjectPtr ConfigurationLoader<mc_rbdyn::S_ObjectPtr>::load(const mc_rtc::Configuration & config)
{
  auto schPointFromConfig = [](const mc_rtc::Configuration & c)
  {
    sch::Point3 p;
    p[0] = c[0];
    p[1] = c[1];
    p[2] = c[2];
    return p;
  };
  std::string type = config("type");
  if(type == "box") { return std::make_shared<sch::S_Box>(config("width"), config("height"), config("depth")); }
  else if(type == "capsule")
  {
    auto p1 = schPointFromConfig(config("p1"));
    auto p2 = schPointFromConfig(config("p2"));
    return std::make_shared<sch::S_Capsule>(p1, p2, config("radius"));
  }
  else if(type == "cone") { return std::make_shared<sch::S_Cone>(config("angle"), config("height")); }
  else if(type == "cylinder")
  {
    auto p1 = schPointFromConfig(config("p1"));
    auto p2 = schPointFromConfig(config("p2"));
    return std::make_shared<sch::S_Cylinder>(p1, p2, config("radius"));
  }
  else if(type == "point") { return std::make_shared<sch::S_Point>(); }
  else if(type == "polyhedron")
  {
    auto out = std::make_shared<sch::S_Polyhedron>();
    out->constructFromFile(config("filename"));
    return out;
  }
  else if(type == "sphere") { return std::make_shared<sch::S_Sphere>(config("radius")); }
  else if(type == "stp-bv")
  {
    auto out = std::make_shared<sch::STP_BV>();
    out->constructFromFile(config("filename"));
    return out;
  }
  else if(type == "stp-bv-p")
  {
    auto out = std::make_shared<sch::STP_BV_P>();
    out->constructFromFile(config("filename"));
    return out;
  }
  else if(type == "superellipsoid")
  {
    return std::make_shared<sch::S_Superellipsoid>(config("a"), config("b"), config("c"), config("epsilon1"),
                                                   config("epsilon2"));
  }
  mc_rtc::log::error_and_throw("Unknown type {} in store S_Object", type);
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::S_ObjectPtr>::save(const mc_rbdyn::S_ObjectPtr & object)
{
  mc_rtc::Configuration out;
  auto schPointToConfig = [&out](const sch::Point3 & p, const std::string & name)
  {
    auto pOut = out.array(name, 3);
    pOut.push(p[0]);
    pOut.push(p[1]);
    pOut.push(p[2]);
  };
  auto type = object->getType();
  if(type == sch::S_Object::TBox)
  {
    auto box = dynamic_cast<sch::S_Box *>(object.get());
    if(!box) { goto failed_cast; }
    double width, height, depth;
    box->getBoxParameters(width, height, depth);
    out.add("type", "box");
    out.add("width", width);
    out.add("height", height);
    out.add("depth", depth);
  }
  else if(type == sch::S_Object::TCapsule)
  {
    auto capsule = dynamic_cast<sch::S_Capsule *>(object.get());
    if(!capsule) { goto failed_cast; }
    out.add("type", "capsule");
    schPointToConfig(capsule->getP1(), "p1");
    schPointToConfig(capsule->getP2(), "p2");
    out.add("radius", capsule->getRadius());
  }
  else if(type == sch::S_Object::TCone)
  {
    auto cone = dynamic_cast<sch::S_Cone *>(object.get());
    if(!cone) { goto failed_cast; }
    out.add("type", "cone");
    out.add("angle", cone->getAngle());
    out.add("height", cone->getHeight());
  }
  else if(type == sch::S_Object::TCylinder)
  {
    auto cylinder = dynamic_cast<sch::S_Cylinder *>(object.get());
    if(!cylinder) { goto failed_cast; }
    out.add("type", "cylinder");
    schPointToConfig(cylinder->getP1(), "p1");
    schPointToConfig(cylinder->getP2(), "p2");
    out.add("radius", cylinder->getRadius());
  }
  else if(type == sch::S_Object::TPoint) { out.add("type", "point"); }
  else if(type == sch::S_Object::TSphere)
  {
    auto sphere = dynamic_cast<sch::S_Sphere *>(object.get());
    if(!sphere) { goto failed_cast; }
    out.add("type", "sphere");
    out.add("radius", sphere->getRadius());
  }
  else if(type == sch::S_Object::TPolyhedron || type == sch::S_Object::TSTP_BV
          || type == sch::S_Object::TSTP_BV_WithPolyhedron)
  {
    mc_rtc::log::error_and_throw("Polyhedron and STP-BV objects cannot be saved this way");
  }
  else if(type == sch::S_Object::TSuperellipsoid)
  {
    auto se = dynamic_cast<sch::S_Superellipsoid *>(object.get());
    if(!se) { goto failed_cast; }
    double a, b, c, epsilon1, epsilon2;
    se->getEllipsoidParameter(a, b, c, epsilon1, epsilon2);
    out.add("type", "superellipsoid");
    out.add("a", a);
    out.add("b", b);
    out.add("c", c);
    out.add("epsilon1", epsilon1);
    out.add("epsilon2", epsilon2);
  }
  else { mc_rtc::log::error_and_throw("New sch-core object type is not handled by this save function"); }
  return out;
failed_cast:
  mc_rtc::log::error_and_throw("Failed to cast the object to its deduced type");
}

mc_rbdyn::RobotModule ConfigurationLoader<mc_rbdyn::RobotModule>::load(const mc_rtc::Configuration & config)
{
  fs::path path((std::string)config("path"));
  std::string name = config("name");
  fs::path urdf_path = [&]()
  {
    if(config.has("urdf_path"))
    {
      fs::path out(static_cast<std::string>(config("urdf_path")));
      if(!out.is_absolute()) { return path / out; }
      return out;
    }
    return path / "urdf" / fmt::format("{}.urdf", name);
  }();
  mc_rbdyn::RobotModule rm(path.string(), name, urdf_path.string());
  config("canonicalParameters", rm._canonicalParameters);
  if(config.has("mb"))
  {
    rm.mb = config("mb");
    rm.mbc = config("mbc");
    rm._bounds = config("bounds");
    rm._visual = static_cast<std::map<std::string, std::vector<rbd::parsers::Visual>>>(config("visuals"));
    rm._collisionTransforms = config("collisionTransforms");
  }
  else
  {
    auto filteredLinks = config("filteredLinks", std::vector<std::string>{});
    auto fixed = config("fixed", false);
    if(!fs::exists(rm.urdf_path))
    {
      mc_rtc::log::error_and_throw("Could not open model for {} at {}", rm.name, rm.urdf_path);
    }
    rm.init(rbd::parsers::from_urdf_file(rm.urdf_path, fixed));
    auto ctfs = config("collisionTransforms", std::map<std::string, sva::PTransformd>{});
    for(const auto & ctf : ctfs)
    {
      if(rm._collisionTransforms.count(ctf.first))
      {
        mc_rtc::log::warning("The collision transform for {} was already loaded from the URDF, the one specified in "
                             "the module will be ignored",
                             ctf.first);
        continue;
      }
      rm._collisionTransforms[ctf.first] = ctf.second;
    }
  }
  if(config.has("accelerationBounds"))
  {
    mc_rbdyn::RobotModule::bounds_t aBounds = config("accelerationBounds");
    if(aBounds.size() != 2) { mc_rtc::log::error_and_throw("accelerationBounds entry should be an array of size 2"); }
    rm._accelerationBounds.resize(2);
    rm._accelerationBounds[0] = aBounds[0];
    rm._accelerationBounds[1] = aBounds[1];
  }
  if(config.has("jerkBounds"))
  {
    mc_rbdyn::RobotModule::bounds_t jBounds = config("jerkBounds");
    if(jBounds.size() != 2) { mc_rtc::log::error_and_throw("jerkBounds entry should be an array of size 2"); }
    rm._jerkBounds.resize(2);
    rm._jerkBounds[0] = jBounds[0];
    rm._jerkBounds[1] = jBounds[1];
  }
  if(config.has("torqueDerivativeBounds"))
  {
    mc_rbdyn::RobotModule::bounds_t tdBounds = config("torqueDerivativeBounds");
    if(tdBounds.size() != 2)
    {
      mc_rtc::log::error_and_throw("torqueDerivativeBounds entry should be an array of size 2");
    }
    rm._torqueDerivativeBounds.resize(2);
    rm._torqueDerivativeBounds[0] = tdBounds[0];
    rm._torqueDerivativeBounds[1] = tdBounds[1];
  }
  /* Default values work fine for those */
  if(config.has("rsdf_dir"))
  {
    fs::path rsdf_dir((std::string)config("rsdf_dir"));
    if(!rsdf_dir.is_absolute()) { rsdf_dir = path / rsdf_dir; }
    rm.rsdf_dir = rsdf_dir.string();
  }
  config("convexHulls", rm._convexHull);
  for(auto & cH : rm._convexHull)
  {
    fs::path chPath(cH.second.second);
    if(!chPath.is_absolute()) { cH.second.second = (path / chPath).string(); }
  }
  config("collisionObjects", rm._collisionObjects);
  config("stpbvHulls", rm._stpbvHull);
  for(auto & sH : rm._stpbvHull)
  {
    fs::path stPath(sH.second.second);
    if(!stPath.is_absolute()) { sH.second.second = (path / stPath).string(); }
  }
  config("flexibilities", rm._flexibility);
  config("forceSensors", rm._forceSensors);
  config("bodySensors", rm._bodySensors);
  config("jointSensors", rm._jointSensors);
  config("springs", rm._springs);
  config("minimalSelfCollisions", rm._minimalSelfCollisions);
  config("commonSelfCollisions", rm._commonSelfCollisions);
  config("default_attitude", rm._default_attitude);

  /* Those cannot be empty */
  config("stance", rm._stance);
  rm.expand_stance();
  if(config.has("ref_joint_order")) { rm._ref_joint_order = config("ref_joint_order"); }
  else { rm.make_default_ref_joint_order(); }

  if(config.has("gripperSafety")) { rm._gripperSafety = config("gripperSafety"); }
  if(config.has("grippers"))
  {
    std::vector<mc_rtc::Configuration> grippers = config("grippers");
    for(auto & g : grippers) { rm._grippers.push_back(loadGripper(g, rm.gripperSafety())); }
  }
  if(config.has("lipmStabilizer")) { rm._lipmStabilizerConfig.load(config("lipmStabilizer")); }

  rm._frames = config("frames", std::vector<mc_rbdyn::RobotModule::FrameDescription>{});

  rm._compoundJoints = config("compoundJoints", mc_rbdyn::CompoundJointConstraintDescriptionVector{});

  return rm;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModule>::save(const mc_rbdyn::RobotModule & rm,
                                                                       bool save_mbc,
                                                                       const std::vector<std::string> & filteredLinks,
                                                                       bool fixed)
{
  mc_rtc::Configuration config;
  config.add("path", rm.path);
  config.add("name", rm.name);
  config.add("urdf_path", relative(rm.urdf_path, rm.path).string());
  config.add("rsdf_dir", relative(rm.rsdf_dir, rm.path).string());
  config.add("canonicalParameters", rm.canonicalParameters());
  if(save_mbc)
  {
    config.add("mb", rm.mb);
    config.add("mbc", rm.mbc);
    config.add("collisionTransforms", rm._collisionTransforms);
    config.add("bounds", rm._bounds);
    config.add("visuals", rm._visual);
  }
  else
  {
    config.add("filteredLinks", filteredLinks);
    config.add("fixed", fixed);
  }
  config.add("frames", rm._frames);
  if(rm._bounds.size() != 6)
  {
    mc_rtc::log::error_and_throw("Wrong number ({}) of _bounds entries in RobotModule", rm._bounds.size());
  }
  if(rm._accelerationBounds.size() != 0 && rm._accelerationBounds.size() != 2)
  {
    mc_rtc::log::error_and_throw("Wrong number ({}) of _accelerationBounds entries in RobotModule",
                                 rm._accelerationBounds.size());
  }
  if(rm._jerkBounds.size() != 0 && rm._jerkBounds.size() != 2)
  {
    mc_rtc::log::error_and_throw("Wrong number ({}) of _jerkBounds entries in RobotModule", rm._jerkBounds.size());
  }
  if(rm._torqueDerivativeBounds.size() != 0 && rm._torqueDerivativeBounds.size() != 2)
  {
    mc_rtc::log::error_and_throw("Wrong number ({}) of _torqueDerivativeBounds entries in RobotModule",
                                 rm._torqueDerivativeBounds.size());
  }
  if(rm._accelerationBounds.size() == 2) { config.add("accelerationBounds", rm._accelerationBounds); }
  if(rm._jerkBounds.size() == 2) { config.add("jerkBounds", rm._jerkBounds); }
  if(rm._torqueDerivativeBounds.size() == 2) { config.add("torqueDerivativeBounds", rm._torqueDerivativeBounds); }
  config.add("stance", rm._stance);
  auto cHs = rm._convexHull;
  for(auto & cH : cHs) { cH.second.second = relative(cH.second.second, rm.path).string(); }
  config.add("convexHulls", cHs);
  auto sHs = rm._convexHull;
  for(auto & sH : sHs) { sH.second.second = relative(sH.second.second, rm.path).string(); }
  config.add("collisionObjects", rm._collisionObjects);
  config.add("stpbvHulls", rm._stpbvHull);
  config.add("flexibilities", rm._flexibility);
  config.add("forceSensors", rm._forceSensors);
  config.add("bodySensors", rm._bodySensors);
  config.add("jointSensors", rm._jointSensors);
  config.add("springs", rm._springs);
  config.add("minimalSelfCollisions", rm._minimalSelfCollisions);
  config.add("commonSelfCollisions", rm._commonSelfCollisions);
  config.add("grippers", rm._grippers);
  config.add("ref_joint_order", rm._ref_joint_order);
  config.add("default_attitude", rm._default_attitude);
  config.add("gripperSafety", rm._gripperSafety);
  config.add("lipmStabilizer", rm._lipmStabilizerConfig);
  if(rm._compoundJoints.size()) { config.add("compoundJoints", rm._compoundJoints); }
  return config;
}

mc_rbdyn::RobotModulePtr ConfigurationLoader<mc_rbdyn::RobotModulePtr>::load(const mc_rtc::Configuration & config)
{
  mc_rbdyn::RobotModule rm = config;
  return std::make_shared<mc_rbdyn::RobotModule>(std::move(rm));
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModulePtr>::save(const mc_rbdyn::RobotModulePtr & rm,
                                                                          bool save_mbc,
                                                                          const std::vector<std::string> & filteredLinks,
                                                                          bool fixed)
{
  return ConfigurationLoader<mc_rbdyn::RobotModule>::save(*rm, save_mbc, filteredLinks, fixed);
}

mc_rbdyn::Contact ConfigurationLoader<mc_rbdyn::Contact>::load(const mc_rtc::Configuration & config,
                                                               const mc_rbdyn::Robots & robots)
{
  const auto r1Index = robotIndexFromConfig(config, robots, "contact", false, "r1Index", "r1");
  const auto r2Index = robotIndexFromConfig(config, robots, "contact", false, "r2Index", "r2", robots.robot(1).name());
  sva::PTransformd X_r2s_r1s = sva::PTransformd::Identity();
  bool isFixed = config("isFixed");
  if(isFixed) { X_r2s_r1s = config("X_r2s_r1s"); }
  std::string r1Surface = config("r1Surface");
  sva::PTransformd X_b_s = robots.robot(r1Index).surface(r1Surface).X_b_s();
  config("X_b_s", X_b_s);
  double friction = config("friction", mc_rbdyn::Contact::defaultFriction);
  int ambiguityId = config("ambiguityId", -1);
  const auto & r1 = robots.robot(r1Index);
  mc_rbdyn::Contact out(robots, r1Index, r2Index, config("r1Surface"), config("r2Surface"), X_r2s_r1s, X_b_s, friction,
                        ambiguityId);
  if(r1.mb().nrDof() == 0) { out = out.swap(robots); }
  return out;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Contact>::save(const mc_rbdyn::Contact & c)
{
  mc_rtc::Configuration config;
  // FIXME saves by index even if the configuration was created by name
  config.add("r1Index", c.r1Index());
  config.add("r2Index", c.r2Index());
  config.add("r1Surface", c.r1Surface()->name());
  config.add("r2Surface", c.r2Surface()->name());
  config.add("X_b_s", c.X_b_s());
  config.add("ambiguityId", c.ambiguityId());
  config.add("isFixed", c.isFixed());
  if(c.isFixed()) { config.add("X_r2s_r1s", c.X_r2s_r1s()); }
  return config;
}

tasks::qp::JointGains ConfigurationLoader<tasks::qp::JointGains>::load(const mc_rtc::Configuration & config)
{
  if(config.has("damping"))
  {
    return tasks::qp::JointGains(config("jointName"), config("stiffness"), config("damping"));
  }
  else { return tasks::qp::JointGains(config("jointName"), config("stiffness")); }
}

mc_rtc::Configuration ConfigurationLoader<tasks::qp::JointGains>::save(const tasks::qp::JointGains & jg)
{
  mc_rtc::Configuration config;
  config.add("jointName", jg.jointName);
  config.add("stiffness", jg.stiffness);
  config.add("damping", jg.damping);
  return config;
}

} // namespace mc_rtc
