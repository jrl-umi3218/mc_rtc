#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/configuration_io.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace
{
// Return relative path to go to "to" from "from"
bfs::path relative(bfs::path to, bfs::path from)
{
  bfs::path::const_iterator fromIter = from.begin();
  bfs::path::const_iterator toIter = to.begin();
  while(fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter))
  {
    ++toIter;
    ++fromIter;
  }
  bfs::path relPath;
  while(fromIter != from.end() && *fromIter != bfs::path("."))
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
  if(type == "rev")
  {
    return rbd::Joint::Type::Rev;
  }
  if(type == "prism")
  {
    return rbd::Joint::Type::Prism;
  }
  if(type == "spherical")
  {
    return rbd::Joint::Type::Spherical;
  }
  if(type == "planar")
  {
    return rbd::Joint::Type::Planar;
  }
  if(type == "cylindrical")
  {
    return rbd::Joint::Type::Cylindrical;
  }
  if(type == "free")
  {
    return rbd::Joint::Type::Free;
  }
  if(type == "fixed")
  {
    return rbd::Joint::Type::Fixed;
  }
  LOG_ERROR(type << " was stored as joint type, cannot comprehend that")
  LOG_ERROR_AND_THROW(std::runtime_error, "Invalid joint type stored")
}

mc_rtc::Configuration ConfigurationLoader<rbd::Joint::Type>::save(const rbd::Joint::Type & type)
{
  mc_rtc::Configuration config;
  std::string typeStr = "";
  switch(type)
  {
    case rbd::Joint::Type::Rev:
      typeStr = "rev";
      break;
    case rbd::Joint::Type::Prism:
      typeStr = "prism";
      break;
    case rbd::Joint::Type::Spherical:
      typeStr = "spherical";
      break;
    case rbd::Joint::Type::Planar:
      typeStr = "planar";
      break;
    case rbd::Joint::Type::Cylindrical:
      typeStr = "cylindrical";
      break;
    case rbd::Joint::Type::Free:
      typeStr = "free";
      break;
    case rbd::Joint::Type::Fixed:
      typeStr = "fixed";
      break;
    default:
      LOG_ERROR("Cannot serialize joint type " << type)
      LOG_ERROR_AND_THROW(std::runtime_error, "Invalid joint type to ConfigurationLoader<rbd::Joint::Type>::save")
  }
  config.add("type", typeStr);
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

mc_rbdyn::Collision ConfigurationLoader<mc_rbdyn::Collision>::load(const mc_rtc::Configuration & config)
{
  return mc_rbdyn::Collision(config("body1"), config("body2"), config("iDist"), config("sDist"), config("damping"));
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Collision>::save(const mc_rbdyn::Collision & c)
{
  mc_rtc::Configuration config;
  config.add("body1", c.body1);
  config.add("body2", c.body2);
  config.add("iDist", c.iDist);
  config.add("sDist", c.sDist);
  config.add("damping", c.damping);
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
  LOG_ERROR("Unknown surface type stored " << type)
  LOG_ERROR_AND_THROW(std::runtime_error, "Invalid surface type stored")
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
  else
  {
    LOG_ERROR("Cannot serialize Surface of type " << s->type())
    LOG_ERROR_AND_THROW(std::runtime_error, "Invalid surface type")
  }
  return config;
}

std::shared_ptr<mc_rbdyn::PlanarSurface> ConfigurationLoader<std::shared_ptr<mc_rbdyn::PlanarSurface>>::load(
    const mc_rtc::Configuration & config)
{
  std::string type = config("type");
  if(type != "planar")
  {
    LOG_ERROR("Tried to deserialize a non-planar surface into a planar surface")
    LOG_ERROR_AND_THROW(std::runtime_error, "Wrong surface types")
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
    LOG_ERROR("Tried to deserialize a non-cylindrical surface into a cylindrical surface")
    LOG_ERROR_AND_THROW(std::runtime_error, "Wrong surface types")
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
    LOG_ERROR("Tried to deserialize a non-gripper surface into a gripper surface")
    LOG_ERROR_AND_THROW(std::runtime_error, "Wrong surface types")
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
  if(isMimic)
  {
    j.makeMimic(config("mimicName"), config("mimicMultiplier"), config("mimicOffset"));
  }
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
    LOG_ERROR_AND_THROW(mc_rtc::Configuration::Exception,
                        "Stored data size (" << data.size() << ") is different from the expected size ("
                                             << 6 * m.cols())
  }
  for(Eigen::DenseIndex i = 0; i < 6; ++i)
  {
    for(Eigen::DenseIndex j = 0; j < m.cols(); ++j)
    {
      m(i, j) = data[m.cols() * i + j];
    }
  }
  return m;
}

mc_rtc::Configuration ConfigurationLoader<Eigen::Matrix<double, 6, Eigen::Dynamic>>::save(
    const Eigen::Matrix<double, 6, Eigen::Dynamic> & m)
{
  mc_rtc::Configuration config;
  config.add("cols", static_cast<int>(m.cols()));
  auto data = config.array("data", 6 * m.cols());
  for(Eigen::DenseIndex i = 0; i < 6; ++i)
  {
    for(Eigen::DenseIndex j = 0; j < m.cols(); ++j)
    {
      data.push(m(i, j));
    }
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

mc_rbdyn::RobotModule::Gripper ConfigurationLoader<mc_rbdyn::RobotModule::Gripper>::load(
    const mc_rtc::Configuration & config)
{
  return {config("name"), config("joints"), config("reverse_limits")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::RobotModule::Gripper>::save(
    const mc_rbdyn::RobotModule::Gripper & rmg)
{
  mc_rtc::Configuration config;
  config.add("name", rmg.name);
  config.add("joints", rmg.joints);
  config.add("reverse_limits", rmg.reverse_limits);
  return config;
}

mc_rbdyn_urdf::Geometry::Box ConfigurationLoader<mc_rbdyn_urdf::Geometry::Box>::load(
    const mc_rtc::Configuration & config)
{
  mc_rbdyn_urdf::Geometry::Box b;
  b.size = config("size");
  return b;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn_urdf::Geometry::Box>::save(const mc_rbdyn_urdf::Geometry::Box & b)
{
  mc_rtc::Configuration config;
  config.add("size", b.size);
  return config;
}

mc_rbdyn_urdf::Geometry::Cylinder ConfigurationLoader<mc_rbdyn_urdf::Geometry::Cylinder>::load(
    const mc_rtc::Configuration & config)
{
  mc_rbdyn_urdf::Geometry::Cylinder c;
  c.radius = config("radius");
  c.length = config("length");
  return c;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn_urdf::Geometry::Cylinder>::save(
    const mc_rbdyn_urdf::Geometry::Cylinder & c)
{
  mc_rtc::Configuration config;
  config.add("radius", c.radius);
  config.add("length", c.length);
  return config;
}

mc_rbdyn_urdf::Geometry::Sphere ConfigurationLoader<mc_rbdyn_urdf::Geometry::Sphere>::load(
    const mc_rtc::Configuration & config)
{
  mc_rbdyn_urdf::Geometry::Sphere s;
  s.radius = config("radius");
  return s;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn_urdf::Geometry::Sphere>::save(
    const mc_rbdyn_urdf::Geometry::Sphere & s)
{
  mc_rtc::Configuration config;
  config.add("radius", s.radius);
  return config;
}

mc_rbdyn_urdf::Geometry::Mesh ConfigurationLoader<mc_rbdyn_urdf::Geometry::Mesh>::load(
    const mc_rtc::Configuration & config)
{
  mc_rbdyn_urdf::Geometry::Mesh m;
  m.filename = static_cast<std::string>(config("filename"));
  m.scale = config("scale");
  return m;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn_urdf::Geometry::Mesh>::save(const mc_rbdyn_urdf::Geometry::Mesh & m)
{
  mc_rtc::Configuration config;
  config.add("filename", m.filename);
  config.add("scale", m.scale);
  return config;
}

mc_rbdyn_urdf::Geometry ConfigurationLoader<mc_rbdyn_urdf::Geometry>::load(const mc_rtc::Configuration & config)
{
  mc_rbdyn_urdf::Geometry geom;
  if(config.has("box"))
  {
    geom.type = mc_rbdyn_urdf::Geometry::Type::BOX;
    mc_rbdyn_urdf::Geometry::Box b = config("box");
    geom.data = b;
  }
  else if(config.has("cylinder"))
  {
    geom.type = mc_rbdyn_urdf::Geometry::Type::CYLINDER;
    mc_rbdyn_urdf::Geometry::Cylinder b = config("cylinder");
    geom.data = b;
  }
  else if(config.has("sphere"))
  {
    geom.type = mc_rbdyn_urdf::Geometry::Type::SPHERE;
    mc_rbdyn_urdf::Geometry::Sphere b = config("sphere");
    geom.data = b;
  }
  else if(config.has("mesh"))
  {
    geom.type = mc_rbdyn_urdf::Geometry::Type::MESH;
    mc_rbdyn_urdf::Geometry::Mesh b = config("mesh");
    geom.data = b;
  }
  return geom;
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn_urdf::Geometry>::save(const mc_rbdyn_urdf::Geometry & geom)
{
  mc_rtc::Configuration config;
  switch(geom.type)
  {
    case mc_rbdyn_urdf::Geometry::Type::BOX:
      config.add("box", boost::get<mc_rbdyn_urdf::Geometry::Box>(geom.data));
      break;
    case mc_rbdyn_urdf::Geometry::Type::CYLINDER:
      config.add("cylinder", boost::get<mc_rbdyn_urdf::Geometry::Cylinder>(geom.data));
      break;
    case mc_rbdyn_urdf::Geometry::Type::SPHERE:
      config.add("sphere", boost::get<mc_rbdyn_urdf::Geometry::Sphere>(geom.data));
      break;
    case mc_rbdyn_urdf::Geometry::Type::MESH:
      config.add("mesh", boost::get<mc_rbdyn_urdf::Geometry::Mesh>(geom.data));
      break;
    default:
      break;
  }
  return config;
}

mc_rbdyn_urdf::Visual ConfigurationLoader<mc_rbdyn_urdf::Visual>::load(const mc_rtc::Configuration & config)
{
  return {config("name"), config("origin"), config("geometry")};
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn_urdf::Visual>::save(const mc_rbdyn_urdf::Visual & vis)
{
  mc_rtc::Configuration config;
  config.add("name", vis.name);
  config.add("origin", vis.origin);
  config.add("geometry", vis.geometry);
  return config;
}

mc_rbdyn::RobotModule ConfigurationLoader<mc_rbdyn::RobotModule>::load(const mc_rtc::Configuration & config)
{
  bfs::path path((std::string)config("path"));
  bfs::path urdf_path((std::string)config("urdf_path"));
  if(!urdf_path.is_absolute())
  {
    urdf_path = path / urdf_path;
  }
  mc_rbdyn::RobotModule rm(path.string(), config("name"), urdf_path.string());
  if(config.has("mb"))
  {
    rm.mb = config("mb");
    rm.mbc = config("mbc");
    rm._bounds = config("bounds");
    rm._visual = config("visuals");
    rm._collisionTransforms = config("collisionTransforms");
  }
  else
  {
    auto filteredLinks = config("filteredLinks", std::vector<std::string>{});
    auto fixed = config("fixed", false);
    std::ifstream ifs(rm.urdf_path);
    if(ifs.is_open())
    {
      std::stringstream urdf;
      urdf << ifs.rdbuf();
      auto res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), fixed, filteredLinks);
      rm.mb = res.mb;
      rm.mbc = res.mbc;
      rm.mbg = res.mbg;
      rm._visual = res.visual;
      rm._collisionTransforms = res.collision_tf;
      rm._bounds = mc_rbdyn::urdf_limits_to_bounds(res.limits);
    }
    else
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "Could not open URDF model for " << rm.name << " at " << rm.urdf_path)
    }
  }

  /* Default values work fine for those */
  if(config.has("rsdf_dir"))
  {
    bfs::path rsdf_dir((std::string)config("rsdf_dir"));
    if(!rsdf_dir.is_absolute())
    {
      rsdf_dir = path / rsdf_dir;
    }
    rm.rsdf_dir = rsdf_dir.string();
  }
  config("convexHulls", rm._convexHull);
  for(auto & cH : rm._convexHull)
  {
    bfs::path chPath(cH.second.second);
    if(!chPath.is_absolute())
    {
      cH.second.second = (path / chPath).string();
    }
  }
  config("stpbvHulls", rm._stpbvHull);
  for(auto & sH : rm._stpbvHull)
  {
    bfs::path stPath(sH.second.second);
    if(!stPath.is_absolute())
    {
      sH.second.second = (path / stPath).string();
    }
  }
  config("flexibilities", rm._flexibility);
  config("forceSensors", rm._forceSensors);
  config("bodySensors", rm._bodySensors);
  config("springs", rm._springs);
  config("minimalSelfCollisions", rm._minimalSelfCollisions);
  config("commonSelfCollisions", rm._commonSelfCollisions);
  config("grippers", rm._grippers);
  config("default_attitude", rm._default_attitude);

  /* Those cannot be empty */
  config("stance", rm._stance);
  rm.expand_stance();
  if(config.has("ref_joint_order"))
  {
    rm._ref_joint_order = config("ref_joint_order");
  }
  else
  {
    rm.make_default_ref_joint_order();
  }
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
  config.add("stance", rm._stance);
  auto cHs = rm._convexHull;
  for(auto & cH : cHs)
  {
    cH.second.second = relative(cH.second.second, rm.path).string();
  }
  config.add("convexHulls", cHs);
  auto sHs = rm._convexHull;
  for(auto & sH : sHs)
  {
    sH.second.second = relative(sH.second.second, rm.path).string();
  }
  config.add("stpbvHulls", rm._stpbvHull);
  config.add("flexibilities", rm._flexibility);
  config.add("forceSensors", rm._forceSensors);
  config.add("bodySensors", rm._bodySensors);
  config.add("springs", rm._springs);
  config.add("minimalSelfCollisions", rm._minimalSelfCollisions);
  config.add("commonSelfCollisions", rm._commonSelfCollisions);
  config.add("grippers", rm._grippers);
  config.add("ref_joint_order", rm._ref_joint_order);
  config.add("default_attitude", rm._default_attitude);
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
  unsigned int r1Index = 0;
  unsigned int r2Index = 1;
  config("r1Index", r1Index);
  config("r2Index", r2Index);
  sva::PTransformd X_r2s_r1s = sva::PTransformd::Identity();
  bool isFixed = config("isFixed");
  if(isFixed)
  {
    X_r2s_r1s = config("X_r2s_r1s");
  }
  std::string r1Surface = config("r1Surface");
  sva::PTransformd X_b_s = robots.robot(r1Index).surface(r1Surface).X_b_s();
  config("X_b_s", X_b_s);
  int ambiguityId = -1;
  config("ambiguityId", ambiguityId);
  return mc_rbdyn::Contact(robots, r1Index, r2Index, config("r1Surface"), config("r2Surface"), X_r2s_r1s, X_b_s,
                           ambiguityId);
}

mc_rtc::Configuration ConfigurationLoader<mc_rbdyn::Contact>::save(const mc_rbdyn::Contact & c)
{
  mc_rtc::Configuration config;
  config.add("r1Index", c.r1Index());
  config.add("r2Index", c.r2Index());
  config.add("r1Surface", c.r1Surface()->name());
  config.add("r2Surface", c.r2Surface()->name());
  config.add("X_b_s", c.X_b_s());
  config.add("ambiguityId", c.ambiguityId());
  config.add("isFixed", c.isFixed());
  if(c.isFixed())
  {
    config.add("X_r2s_r1s", c.X_r2s_r1s());
  }
  return config;
}

tasks::qp::JointGains ConfigurationLoader<tasks::qp::JointGains>::load(const mc_rtc::Configuration & config)
{
  if(config.has("damping"))
  {
    return tasks::qp::JointGains(config("jointName"), config("stiffness"), config("damping"));
  }
  else
  {
    return tasks::qp::JointGains(config("jointName"), config("stiffness"));
  }
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
