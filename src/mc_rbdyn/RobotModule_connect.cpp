/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/* This file has the implementation of RobotModule::connect and numerous related utilities */

#include <mc_rbdyn/RobotModule.h>

#include <mc_rbdyn/configuration_io.h>

#include <RBDyn/FK.h>
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <tinyxml2.h>

namespace mc_rbdyn
{

namespace
{

std::string make_temporary_path(const std::string & prefix)
{
  auto tmp = bfs::temp_directory_path();
  auto pattern = fmt::format("{}-%%%%-%%%%-%%%%-%%%%", prefix);
  auto out = tmp / bfs::unique_path(pattern);
  bfs::create_directories(out);
  return out.string();
}

template<std::string RobotModule::*member, typename GetDefault>
void set_or_default(mc_rbdyn::RobotModule & out, const std::string & value, GetDefault && get_default)
{
  if(value.size() == 0) { out.*member = get_default(); }
  else { out.*member = value; }
}

std::string prefixed_or_mapping(const std::string & ref,
                                const RobotModule::ConnectionParameters::mapping_t & mapping,
                                const std::string & prefix)
{
  auto it = mapping.find(ref);
  if(it != mapping.end()) { return it->second; }
  return fmt::format("{}{}", prefix, ref);
}

Eigen::Vector3d axisFromJoint(const rbd::Joint & j)
{
  using Type = rbd::Joint::Type;
  switch(j.type())
  {
    case Type::Rev:
    case Type::Cylindrical:
      return j.direction() * j.motionSubspace().col(0).head<3>();
    case Type::Prism:
      return j.direction() * j.motionSubspace().col(0).tail(3);
    default: // The joint axis is irrelevant for every other joint
      return Eigen::Vector3d::UnitZ();
  }
}

template<typename SurfaceFileCallback>
void processSurfaces(const std::string & rsdf_dir_in, const std::string & rsdf_dir_out, SurfaceFileCallback && callback)
{
  bfs::path pathIn(rsdf_dir_in);
  bfs::path pathOut(rsdf_dir_out);
  if(!bfs::exists(pathIn) || !bfs::is_directory(pathIn)) { return; }
  std::vector<bfs::path> files;
  std::copy(bfs::directory_iterator(pathIn), bfs::directory_iterator(), std::back_inserter(files));
  for(const auto & f : files)
  {
    if(f.extension() == ".rsdf")
    {
      bfs::path out = pathOut / f.leaf();
      bfs::copy(f, out);
      callback(out.string());
    }
  }
}

} // namespace

RobotModule RobotModule::connect(const mc_rbdyn::RobotModule & other,
                                 const std::string & this_body,
                                 const std::string & other_body,
                                 const std::string & prefix,
                                 const ConnectionParameters & params) const
{
  mc_rtc::log::info("Merging {} and {} RobotModule", name, other.name);

  // First we copy this into the output RobotModule
  auto out = *this;

  // Handle general options
#define SET_OR_DEFAULT(NAME, CALLBACK) set_or_default<&RobotModule::NAME>(out, params.NAME, CALLBACK)
#define SET_OR_DEFAULT_DIRECTORY(NAME, CALLBACK) \
  SET_OR_DEFAULT(NAME, CALLBACK);                \
  if(!bfs::exists(out.NAME)) { bfs::create_directories(out.NAME); }
  SET_OR_DEFAULT(name, ([&, this]() { return fmt::format("{}_{}_{}", this->name, prefix, other.name); }));
  SET_OR_DEFAULT_DIRECTORY(path, ([&]() { return make_temporary_path(out.name); }));
  SET_OR_DEFAULT(urdf_path, ([&]() { return (bfs::path(out.path) / "urdf" / (out.name + ".urdf")).string(); }));
  auto urdf_dir = bfs::path(out.urdf_path).parent_path();
  if(!bfs::exists(urdf_dir)) { bfs::create_directories(urdf_dir); }
  SET_OR_DEFAULT_DIRECTORY(rsdf_dir, ([&]() { return (bfs::path(out.path) / "rsdf" / out.name).string(); }));
  SET_OR_DEFAULT_DIRECTORY(calib_dir, ([&]() { return (bfs::path(out.path) / "calib").string(); }));
  if(!params.useGripperSafetyFromThis) { out._gripperSafety = other._gripperSafety; }
  if(!params.useLIPMStabilizerConfigFromThis) { out._lipmStabilizerConfig = other._lipmStabilizerConfig; }
#undef SET_OR_DEFAULT
#undef SET_OR_DEFAULT_DIRECTORY

  // A few helpers to handle name remapping
  auto bodyName = [&](const std::string & body) { return prefixed_or_mapping(body, params.bodyMapping, prefix); };
  auto jointName = [&](const std::string & joint) { return prefixed_or_mapping(joint, params.jointMapping, prefix); };
  auto convexName = [&](const std::string & convex)
  { return prefixed_or_mapping(convex, params.convexMapping, prefix); };
  auto gripperName = [&](const std::string & gripper)
  { return prefixed_or_mapping(gripper, params.gripperMapping, prefix); };
  auto surfaceName = [&](const std::string & surface)
  { return prefixed_or_mapping(surface, params.surfaceMapping, prefix); };
  auto forceSensorName = [&](const std::string & forceSensor)
  { return prefixed_or_mapping(forceSensor, params.forceSensorMapping, prefix); };
  auto bodySensorName = [&](const std::string & bodySensor)
  { return prefixed_or_mapping(bodySensor, params.bodySensorMapping, prefix); };
  auto deviceName = [&](const std::string & device)
  { return prefixed_or_mapping(device, params.deviceMapping, prefix); };

  // Build a new MultiBodyGraph from this and other
  auto & mbg = out.mbg;
  // Add all the bodies from other
  for(size_t i = 0; i < other.mb.bodies().size(); ++i)
  {
    const auto & body = other.mb.body(static_cast<int>(i));
    mbg.addBody(rbd::Body{body.inertia(), bodyName(body.name())});
  }
  // Add all joints except the root joint from other
  for(size_t i = 1; i < other.mb.joints().size(); ++i)
  {
    const auto & joint = other.mb.joint(static_cast<int>(i));
    rbd::Joint j{joint.type(), axisFromJoint(joint), joint.forward(), jointName(joint.name())};
    if(joint.isMimic()) { j.makeMimic(jointName(joint.mimicName()), joint.mimicMultiplier(), joint.mimicOffset()); }
    mbg.addJoint(j);
  }
  // Add all connections except the root<->world connection
  for(size_t i = 1; i < other.mb.joints().size(); ++i)
  {
    auto predIdx = other.mb.predecessor(static_cast<int>(i));
    const auto & predBody = other.mb.body(predIdx);
    auto succIdx = other.mb.successor(static_cast<int>(i));
    const auto & succBody = other.mb.body(succIdx);
    const auto & X_pred_joint = other.mb.transform(static_cast<int>(i));
    const auto & X_joint_succ = other.mbc.jointConfig[i];
    auto X_succ_joint = X_joint_succ.inv();
    mbg.linkBodies(bodyName(predBody.name()), X_pred_joint, bodyName(succBody.name()), X_succ_joint,
                   jointName(other.mb.joint(static_cast<int>(i)).name()));
  }
  std::string connectJointName = params.jointName;
  if(connectJointName.size() == 0) { connectJointName = fmt::format("{}_connect_{}_{}", name, prefix, other.name); }
  rbd::Joint connectJoint{params.jointType, params.jointAxis, params.jointForward, connectJointName};
  mbg.addJoint(connectJoint);
  mbg.linkBodies(this_body, params.X_this_connection, bodyName(other_body), params.X_other_connection,
                 connectJointName);

  /** Create a new MultiBodyGraph which has the same base as this */
  const auto & X_0_b0 = mbc.bodyPosW[0];
  const auto & X_0_j0 = mb.transform(0);
  auto X_b0_j0 = mbc.jointConfig[0] * X_0_j0 * X_0_b0.inv();
  out.mb = mbg.makeMultiBody(mb.body(0).name(), mb.joint(0).type(), axisFromJoint(mb.joint(0)), X_0_j0, X_b0_j0);
  out.mbc = rbd::MultiBodyConfig(out.mb);
  out.mbc.zero(out.mb);
  rbd::forwardKinematics(out.mb, out.mbc);

  // Note that if we get to this point there was no duplicate joint/body name

  /** Check and add the provided bounds for the connection joint */
  for(size_t i = 0; i < 2; ++i)
  {
    if(params.jointLimits[i].size() != static_cast<size_t>(connectJoint.params()))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "You provided invalid position limits for the connection joint, expected {} but got {}",
          connectJoint.params(), params.jointLimits[i].size());
    }
    out._bounds[i][connectJointName] = params.jointLimits[i];
  }
  for(size_t i = 2; i < 6; ++i)
  {
    if(params.jointLimits[i].size() != static_cast<size_t>(connectJoint.dof()))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "You provided invalid velocity/torque limits for the connection joint, expected {} but got {}",
          connectJoint.params(), params.jointLimits[i].size());
    }
    out._bounds[i][connectJointName] = params.jointLimits[i];
  }
  for(size_t i = 0; i < 2; ++i)
  {
    const auto & limit = params.jointAccelerationLimits[i];
    if(limit.size() != 0 && limit.size() != static_cast<size_t>(connectJoint.dof()))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "You provided invalid joint acceleration limits for the connection joint, expected {} but got {}",
          connectJoint.dof(), limit.size());
    }
    if(params.jointAccelerationLimits[i].size() == 0) { continue; }
    if(out._accelerationBounds.size() < 2) { out._accelerationBounds.resize(2); }
    out._accelerationBounds[i][connectJointName] = params.jointAccelerationLimits[i];
  }
  for(size_t i = 0; i < 2; ++i)
  {
    const auto & limit = params.jointTorqueDerivativeLimits[i];
    if(limit.size() != 0 && limit.size() != static_cast<size_t>(connectJoint.dof()))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "You provided invalid joint torqueDerivative limits for the connection joint, expected {} but got {}",
          connectJoint.dof(), limit.size());
    }
    if(params.jointTorqueDerivativeLimits[i].size() == 0) { continue; }
    if(out._torqueDerivativeBounds.size() < 2) { out._torqueDerivativeBounds.resize(2); }
    out._torqueDerivativeBounds[i][connectJointName] = params.jointTorqueDerivativeLimits[i];
  }
  for(size_t i = 0; i < 2; ++i)
  {
    const auto & limit = params.jointJerkLimits[i];
    if(limit.size() != 0 && limit.size() != static_cast<size_t>(connectJoint.dof()))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "You provided invalid joint jerk limits for the connection joint, expected {} but got {}", connectJoint.dof(),
          limit.size());
    }
    if(params.jointJerkLimits[i].size() == 0) { continue; }
    if(out._jerkBounds.size() < 2) { out._jerkBounds.resize(2); }
    out._jerkBounds[i][connectJointName] = params.jointJerkLimits[i];
  }

  /** Check the stance configuration for the connection joint */
  if(params.jointStance.size() != static_cast<size_t>(connectJoint.params()))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You provided an invalid configuration for the connection joint stance, expected {} params but got {}",
        connectJoint.params(), params.jointStance.size());
  }

  /** Update all bounds */
  for(size_t i = 1; i < other.mb.joints().size(); ++i)
  {
    const auto & jName = other.mb.joint(static_cast<int>(i)).name();
    const auto & newName = jointName(jName);
    auto updateBounds = [&](size_t jMax, const bounds_t & boundsIn, bounds_t & boundsOut)
    {
      for(size_t j = 0; j < jMax; ++j)
      {
        const auto & bound = boundsIn[j];
        auto it = bound.find(jName);
        if(it != bound.end()) { boundsOut[j][newName] = it->second; }
      }
    };
    updateBounds(6, other._bounds, out._bounds);
    if(other._accelerationBounds.size())
    {
      if(!out._accelerationBounds.size()) { out._accelerationBounds.resize(2); }
      updateBounds(2, other._accelerationBounds, out._accelerationBounds);
    }
    if(other._torqueDerivativeBounds.size())
    {
      if(!out._torqueDerivativeBounds.size()) { out._torqueDerivativeBounds.resize(2); }
      updateBounds(2, other._torqueDerivativeBounds, out._torqueDerivativeBounds);
    }
    if(other._jerkBounds.size())
    {
      if(!out._jerkBounds.size()) { out._jerkBounds.resize(2); }
      updateBounds(2, other._jerkBounds, out._jerkBounds);
    }
  }

  /** Update visual and collision visual maps */
  auto updateVisualMap = [&](const VisualMap & visualsIn, VisualMap & visualsOut)
  {
    for(const auto & v : visualsIn) { visualsOut[bodyName(v.first)] = v.second; }
  };
  updateVisualMap(other._visual, out._visual);
  updateVisualMap(other._collision, out._collision);

  /** Save the URDF */
  {
    rbd::parsers::ParserResult result;
    result.mb = out.mb;
    result.mbc = out.mbc;
    result.mbg = out.mbg;
    result.visual = out._visual;
    result.collision = out._collision;
    result.limits.lower = out._bounds[0];
    result.limits.upper = out._bounds[1];
    result.limits.velocity = out._bounds[3];
    result.limits.torque = out._bounds[5];
    result.name = out.name;
    std::ofstream ofs(out.urdf_path);
    ofs << rbd::parsers::to_urdf(result);
  }

  /** Update stance */
  for(const auto & s : other._stance)
  {
    const auto & jName = s.first;
    if(jName == "Root") { continue; }
    const auto & jConfig = s.second;
    out._stance[jointName(jName)] = jConfig;
  }

  /** Update convex/stpbv hulls/sch objects */
  auto updateHulls = [&](const std::map<std::string, std::pair<std::string, std::string>> & hullsIn,
                         std::map<std::string, std::pair<std::string, std::string>> & hullsOut)
  {
    for(const auto & h : hullsIn)
    {
      const auto & cName = h.first;
      const auto & bName = h.second.first;
      const auto & cPath = h.second.second;
      const auto & newConvexName = convexName(cName);
      if(hullsOut.count(newConvexName))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "Convex hull name collision during connection, would add {} but it already exists in the original "
            "module, provide or change the convex mapping",
            newConvexName);
      }
      hullsOut[newConvexName] = {bodyName(bName), cPath};
    }
  };
  updateHulls(other._convexHull, out._convexHull);
  updateHulls(other._stpbvHull, out._stpbvHull);
  for(const auto & co : other._collisionObjects)
  {
    const auto & co_name = co.first;
    const auto & new_name = convexName(co_name);
    if(_collisionObjects.count(new_name))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Collision objects name collision during connection, would add {} but it already exists in the original "
          "module, provide or change the convex mapping",
          new_name);
    }
    out._collisionObjects[new_name] = {co.second.first, S_ObjectPtr(co.second.second->clone())};
  }

  /** Update collision transforms */
  for(const auto & ct : other._collisionTransforms) { out._collisionTransforms[convexName(ct.first)] = ct.second; }

  /** Update flexibility */
  for(const auto & f : other._flexibility) { out._flexibility.push_back({jointName(f.jointName), f.K, f.C, f.O}); }

  /** Update force sensors */
  for(const auto & fs : other._forceSensors)
  {
    const auto & fsName = forceSensorName(fs.name());
    auto it = std::find_if(out._forceSensors.begin(), out._forceSensors.end(),
                           [&fsName](const mc_rbdyn::ForceSensor & fs) { return fs.name() == fsName; });
    if(it != out._forceSensors.end())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Force sensor name collision during connection, would add {} but it already exists in the original "
          "module, provide or change the force sensor mapping",
          fsName);
    }
    out._forceSensors.emplace_back(fsName, bodyName(fs.parent()), fs.X_p_f());
  }

  /** Update body sensors */
  for(const auto & bs : other._bodySensors)
  {
    const auto & bsName = bodySensorName(bs.name());
    auto it = std::find_if(out._bodySensors.begin(), out._bodySensors.end(),
                           [&bsName](const mc_rbdyn::BodySensor & bs) { return bs.name() == bsName; });
    if(it != out._bodySensors.end())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Body sensor name collision during connection, would add {} but it already exists in the original "
          "module, provide or change the body sensor mapping",
          bsName);
    }
    out._bodySensors.emplace_back(bsName, bodyName(bs.parent()), bs.X_p_s());
  }

  /** Update springs */
  for(size_t i = 0; i < other._springs.springsBodies.size(); ++i)
  {
    const auto & springBody = bodyName(other._springs.springsBodies[i]);
    const auto & afterSpringBody = bodyName(other._springs.afterSpringsBodies[i]);
    std::vector<std::string> springJoints;
    for(const auto & j : other._springs.springsJoints[i]) { springJoints.push_back(jointName(j)); }
    out._springs.springsBodies.push_back(springBody);
    out._springs.afterSpringsBodies.push_back(afterSpringBody);
    out._springs.springsJoints.push_back(springJoints);
  }

  /** Update self-collisions set */
  auto updateSelfCollisions =
      [&](const std::vector<mc_rbdyn::Collision> & colsIn, std::vector<mc_rbdyn::Collision> & colsOut)
  {
    for(const auto & c : colsIn)
    {
      colsOut.push_back({convexName(c.body1), convexName(c.body2), c.iDist, c.sDist, c.damping});
    }
  };
  updateSelfCollisions(other._minimalSelfCollisions, out._minimalSelfCollisions);
  updateSelfCollisions(other._commonSelfCollisions, out._commonSelfCollisions);

  /** Merge the two ref_joint_order */
  if(connectJoint.dof() > 0) { out._ref_joint_order.push_back(connectJointName); }
  for(const auto & j : other._ref_joint_order) { out._ref_joint_order.push_back(jointName(j)); }

  /** Update grippers */
  for(const auto & g : other._grippers)
  {
    const auto & name = gripperName(g.name);
    auto it =
        std::find_if(out._grippers.begin(), out._grippers.end(), [&name](const Gripper & g) { return g.name == name; });
    if(it != out._grippers.end())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Gripper name collision during connection, would add {} but it already exists in the original "
          "module, provide or change the gripper mapping",
          name);
    }
    std::vector<std::string> joints;
    for(const auto & j : g.joints) { joints.push_back(jointName(j)); }
    if(g.safety())
    {
      if(g.mimics()) { out._grippers.emplace_back(name, joints, g.reverse_limits, *g.safety(), *g.mimics()); }
      else { out._grippers.emplace_back(name, joints, g.reverse_limits, *g.safety()); }
    }
    else { out._grippers.emplace_back(name, joints, g.reverse_limits); }
  }

  /** Update compound joint description */
  for(const auto & cj : other._compoundJoints)
  {
    out._compoundJoints.push_back({jointName(cj.j1), jointName(cj.j2), cj.p1, cj.p2});
  }

  /** Update devices */
  for(const auto & d : other._devices)
  {
    auto name = deviceName(d->name());
    auto it = std::find_if(out._devices.begin(), out._devices.end(),
                           [&name](const DevicePtr & d) { return d->name() == name; });
    if(it != out._devices.end())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Device name collision during connection, would add {} but it already exists in the original "
          "module, provide or change the device mapping",
          name);
    }
    out._devices.emplace_back(d->clone());
    out._devices.back()->name(name);
  }

  /** Update surfaces, for this we:
   * - Copy all surfaces from this to out.rsdf_dir
   * - Update the link property in every surface from other and copy the new file to out.rsdf_dir
   */
  processSurfaces(rsdf_dir, out.rsdf_dir, [](const std::string &) {});
  processSurfaces(other.rsdf_dir, out.rsdf_dir,
                  [&](const std::string & rsdf)
                  {
                    std::ifstream ifs(rsdf);
                    std::stringstream ss;
                    ss << ifs.rdbuf();
                    tinyxml2::XMLDocument doc;
                    doc.Parse(ss.str().c_str());
                    auto root = doc.FirstChildElement("robot");
                    auto updateSurfaces = [&](const char * type)
                    {
                      auto surface = root->FirstChildElement(type);
                      while(surface)
                      {
                        std::string name = surface->Attribute("name");
                        surface->SetAttribute("name", surfaceName(name).c_str());
                        std::string link = surface->Attribute("link");
                        surface->SetAttribute("link", bodyName(link).c_str());
                        surface = surface->NextSiblingElement(type);
                      }
                    };
                    updateSurfaces("planar_surface");
                    updateSurfaces("cylindrical_surface");
                    updateSurfaces("gripper_surface");
                    doc.SaveFile(rsdf.c_str());
                    auto p = bfs::path(rsdf);
                    bfs::rename(p, p.parent_path() / (prefix + p.leaf().string()));
                  });

  // Generate a module file so that the generated RobotModule can be re-used
  {
    std::string module_yaml = fmt::format("{}/{}.yaml", out.path, out.name);
    auto yaml = mc_rtc::ConfigurationLoader<mc_rbdyn::RobotModule>::save(out, false, {}, out.mb.joint(0).dof() == 0);
    yaml.save(module_yaml);
    out._parameters = {"json", module_yaml};
    mc_rtc::log::info("Connection done, result module in: {}", module_yaml);
  }

  return out;
}

RobotModule RobotModule::disconnect(const mc_rbdyn::RobotModule & other,
                                    const std::string & this_body,
                                    const std::string & other_body,
                                    const std::string & prefix,
                                    const ConnectionParameters & params) const
{
  mc_rtc::log::info("Disconnecting {} from {} RobotModule", other.name, name);

  // A few helpers to handle name remapping
  auto bodyName = [&](const std::string & body) { return prefixed_or_mapping(body, params.bodyMapping, prefix); };
  auto jointName = [&](const std::string & joint) { return prefixed_or_mapping(joint, params.jointMapping, prefix); };
  auto convexName = [&](const std::string & convex)
  { return prefixed_or_mapping(convex, params.convexMapping, prefix); };
  auto gripperName = [&](const std::string & gripper)
  { return prefixed_or_mapping(gripper, params.gripperMapping, prefix); };
  auto forceSensorName = [&](const std::string & forceSensor)
  { return prefixed_or_mapping(forceSensor, params.forceSensorMapping, prefix); };
  auto bodySensorName = [&](const std::string & bodySensor)
  { return prefixed_or_mapping(bodySensor, params.bodySensorMapping, prefix); };
  auto deviceName = [&](const std::string & device)
  { return prefixed_or_mapping(device, params.deviceMapping, prefix); };
  std::string connection_joint = "";
  {
    const auto & bIndexByName = mb.bodyIndexByName();
    auto this_it = bIndexByName.find(this_body);
    auto other_it = bIndexByName.find(bodyName(other_body));
    if(this_it == bIndexByName.end() || other_it == bIndexByName.end())
    {
      mc_rtc::log::error("Failed to disconnect {} from {} RobotModule, {} or {} is not in the module", other.name, name,
                         this_body, bodyName(other_body));
      return *this;
    }
    int this_body_idx = this_it->second;
    int other_body_idx = other_it->second;
    const auto & pred = mb.predecessors();
    const auto & succ = mb.successors();
    for(size_t i = 0; i < pred.size(); ++i)
    {
      if(pred[i] == this_body_idx && succ[i] == other_body_idx)
      {
        connection_joint = mb.joint(static_cast<int>(i)).name();
        break;
      }
    }
    if(connection_joint.size() == 0)
    {
      mc_rtc::log::error("Cannot disconnect {} from {} RobotModule, looks like they are not connected via {} and {}",
                         other.name, name, this_body, other_body);
    }
    // TODO This should also check that there is nothing not belonging to \ref
    // other that is connected under other_body, e.g. if I create the
    // connection A -> B -> C and later remove only B, the current
    // implementation would produce a broken module as it would remove things
    // links and joints from C but keep some data around that refers to C.
    // The correct approach would be to disconnect C and then B.
  }

  // We first copy this module and we will start the cleanup
  auto out = *this;

  // Name requires specific handling
  if(params.name != "") { out.name = params.name; }
  else
  {
    std::string search = fmt::format("{}_{}", prefix, other.name);
    auto idx = out.name.find(search);
    if(idx != std::string::npos) { out.name.erase(idx, search.size()); }
  }

  // Handle other general options
#define SET_OR_DEFAULT(NAME, CALLBACK) set_or_default<&RobotModule::NAME>(out, params.NAME, CALLBACK)
#define SET_OR_DEFAULT_DIRECTORY(NAME, CALLBACK) \
  SET_OR_DEFAULT(NAME, CALLBACK);                \
  if(!bfs::exists(out.NAME)) { bfs::create_directories(out.NAME); }
  SET_OR_DEFAULT_DIRECTORY(path, ([&]() { return make_temporary_path(out.name); }));
  SET_OR_DEFAULT(urdf_path, ([&]() { return (bfs::path(out.path) / "urdf" / (out.name + ".urdf")).string(); }));
  auto urdf_dir = bfs::path(out.urdf_path).parent_path();
  if(!bfs::exists(urdf_dir)) { bfs::create_directories(urdf_dir); }
  SET_OR_DEFAULT_DIRECTORY(rsdf_dir, ([&]() { return (bfs::path(out.path) / "rsdf" / out.name).string(); }));
  SET_OR_DEFAULT_DIRECTORY(calib_dir, ([&]() { return (bfs::path(out.path) / "calib").string(); }));
#undef SET_OR_DEFAULT
#undef SET_OR_DEFAULT_DIRECTORY

  /** Remove the connection joint and all connected bodies/joints */
  out.mbg.removeJoint(out.mb.body(0).name(), connection_joint);
  /** Create a new MultiBodyGraph which has the same base as this */
  const auto & X_0_b0 = mbc.bodyPosW[0];
  const auto & X_0_j0 = mb.transform(0);
  auto X_b0_j0 = mbc.jointConfig[0] * X_0_j0 * X_0_b0.inv();
  out.mb = out.mbg.makeMultiBody(mb.body(0).name(), mb.joint(0).type(), axisFromJoint(mb.joint(0)), X_0_j0, X_b0_j0);
  out.mbc = rbd::MultiBodyConfig(out.mb);
  out.mbc.zero(out.mb);
  rbd::forwardKinematics(out.mb, out.mbc);

  /** Erase all extra bounds */
  auto eraseBounds = [&](const std::string & jName)
  {
    for(size_t i = 0; i < 2; ++i)
    {
      out._bounds[i].erase(jName);
      if(out._accelerationBounds.size()) { out._accelerationBounds[i].erase(jName); }
      if(out._torqueDerivativeBounds.size()) { out._torqueDerivativeBounds[i].erase(jName); }
      if(out._jerkBounds.size()) { out._jerkBounds[i].erase(jName); }
    }
    for(size_t i = 2; i < 6; ++i) { out._bounds[i].erase(jName); }
    // Remove connection joint from stance
    out._stance.erase(jName);
  };
  eraseBounds(connection_joint);
  for(const auto & j : other.mb.joints()) { eraseBounds(jointName(j.name())); }

  /** Update the visual and collision maps */
  auto updateVisualMap = [&](const VisualMap & in, VisualMap & out)
  {
    for(const auto & v : in) { out.erase(bodyName(v.first)); }
  };
  updateVisualMap(other._visual, out._visual);
  updateVisualMap(other._collision, out._collision);

  /** Save the URDF */
  {
    rbd::parsers::ParserResult result;
    result.mb = out.mb;
    result.mbc = out.mbc;
    result.mbg = out.mbg;
    result.visual = out._visual;
    result.collision = out._collision;
    result.limits.lower = out._bounds[0];
    result.limits.upper = out._bounds[1];
    result.limits.velocity = out._bounds[3];
    result.limits.torque = out._bounds[5];
    result.name = out.name;
    std::ofstream ofs(out.urdf_path);
    ofs << rbd::parsers::to_urdf(result);
  }

  /** Update stance */
  for(const auto & s : other._stance)
  {
    const auto & jName = s.first;
    if(jName == "Root") { continue; }
    out._stance.erase(jointName(jName));
  }

  /** Update convex/stpbv hulls/sch objects */
  auto updateHulls = [&](const std::map<std::string, std::pair<std::string, std::string>> & hullsIn,
                         std::map<std::string, std::pair<std::string, std::string>> & hullsOut)
  {
    for(const auto & h : hullsIn) { hullsOut.erase(convexName(h.first)); }
  };
  updateHulls(other._convexHull, out._convexHull);
  updateHulls(other._stpbvHull, out._stpbvHull);
  for(const auto & co : other._collisionObjects) { out._collisionObjects.erase(convexName(co.first)); }

  /** Update collision transforms */
  for(const auto & ct : other._collisionTransforms) { out._collisionTransforms.erase(convexName(ct.first)); }

  /** Update flexibilities */
  for(const auto & flex : other._flexibility)
  {
    auto it = std::find_if(out._flexibility.begin(), out._flexibility.end(),
                           [&](const Flexibility & f) { return f.jointName == jointName(flex.jointName); });
    if(it != out._flexibility.end()) { out._flexibility.erase(it); }
  }

  /** Update force sensors */
  for(const auto & fs : other._forceSensors)
  {
    for(auto it = out._forceSensors.begin(); it != out._forceSensors.end(); ++it)
    {
      if(it->name() == forceSensorName(fs.name()))
      {
        out._forceSensors.erase(it);
        break;
      }
    }
  }

  /** Update body sensors */
  for(const auto & fs : other._bodySensors)
  {
    for(auto it = out._bodySensors.begin(); it != out._bodySensors.end(); ++it)
    {
      if(it->name() == bodySensorName(fs.name()))
      {
        out._bodySensors.erase(it);
        break;
      }
    }
  }

  /** Update springs */
  for(size_t i = 0; i < other._springs.springsBodies.size(); ++i)
  {
    const auto & springBody = bodyName(other._springs.springsBodies[i]);
    auto it = std::find_if(out._springs.springsBodies.begin(), out._springs.springsBodies.end(),
                           [&](const std::string & b) { return b == springBody; });
    if(it == out._springs.springsBodies.end()) { continue; }
    auto idx = std::distance(out._springs.springsBodies.begin(), it);
    out._springs.springsBodies.erase(it);
    out._springs.springsJoints.erase(out._springs.springsJoints.begin() + idx);
    out._springs.afterSpringsBodies.erase(out._springs.afterSpringsBodies.begin() + idx);
  }

  /** Update self-collisions set */
  auto updateSelfCollisions =
      [&](const std::vector<mc_rbdyn::Collision> & colsIn, std::vector<mc_rbdyn::Collision> & colsOut)
  {
    for(const auto & c : colsIn)
    {
      auto it = std::find_if(colsOut.begin(), colsOut.end(),
                             [&](const Collision & col)
                             { return col.body1 == convexName(c.body1) && col.body2 == convexName(c.body2); });
      if(it != colsOut.end()) { colsOut.erase(it); }
    }
  };
  updateSelfCollisions(other._minimalSelfCollisions, out._minimalSelfCollisions);
  updateSelfCollisions(other._commonSelfCollisions, out._commonSelfCollisions);

  /** Update ref joint order */
  auto removeFromRefJointOrder = [&](const std::string & jName)
  {
    auto it = std::find(out._ref_joint_order.begin(), out._ref_joint_order.end(), jName);
    if(it != out._ref_joint_order.end()) { out._ref_joint_order.erase(it); }
  };
  removeFromRefJointOrder(connection_joint);
  for(const auto & j : other._ref_joint_order) { removeFromRefJointOrder(jointName(j)); }

  /** Update grippers */
  for(const auto & g : other._grippers)
  {
    const auto & name = gripperName(g.name);
    auto it =
        std::find_if(out._grippers.begin(), out._grippers.end(), [&name](const Gripper & g) { return g.name == name; });
    if(it != out._grippers.end()) { out._grippers.erase(it); }
  }

  /** Update compound joint description */
  for(const auto & cj : other._compoundJoints)
  {
    auto it = std::find_if(out._compoundJoints.begin(), out._compoundJoints.end(),
                           [&](const CompoundJointConstraintDescription & cjIn)
                           { return cjIn.j1 == jointName(cj.j1) && cjIn.j2 == jointName(cj.j2); });
    if(it != out._compoundJoints.end()) { out._compoundJoints.erase(it); }
  }

  /** Update devices */
  for(const auto & d : other._devices)
  {
    auto name = deviceName(d->name());
    auto it = std::find_if(out._devices.begin(), out._devices.end(),
                           [&name](const DevicePtr & d) { return d->name() == name; });
    if(it != out._devices.end()) { out._devices.erase(it); }
  }

  /** Update surfaces we simply copy all files except those that are also in other.rsdf_dir */
  auto copySurfaces = [&]()
  {
    bfs::path this_rsdf_dir(rsdf_dir);
    bfs::path other_rsdf_dir(other.rsdf_dir);
    bfs::path out_rsdf_dir(out.rsdf_dir);
    if(!bfs::exists(this_rsdf_dir) || !bfs::is_directory(this_rsdf_dir)) { return; }
    std::vector<bfs::path> this_files;
    std::copy(bfs::directory_iterator(this_rsdf_dir), bfs::directory_iterator(), std::back_inserter(this_files));
    std::vector<bfs::path> other_files;
    if(bfs::exists(other_rsdf_dir) && bfs::is_directory(other_rsdf_dir))
    {
      std::copy(bfs::directory_iterator(other_rsdf_dir), bfs::directory_iterator(), std::back_inserter(other_files));
    }
    for(const auto & f : this_files)
    {
      if(f.extension() != ".rsdf") { continue; }
      auto it = std::find_if(other_files.begin(), other_files.end(),
                             [&](const bfs::path & other_f) { return f.leaf() == other_f.leaf(); });
      // Skip the copy if the file is from other
      if(it != other_files.end()) { continue; }
      bfs::path out = out_rsdf_dir / f.leaf();
      bfs::copy(f, out);
    }
  };
  copySurfaces();

  // Generate a module file so that the generated RobotModule can be re-used
  {
    std::string module_yaml = fmt::format("{}/{}.yaml", out.path, out.name);
    auto yaml = mc_rtc::ConfigurationLoader<mc_rbdyn::RobotModule>::save(out, false, {}, out.mb.joint(0).dof() == 0);
    yaml.save(module_yaml);
    out._parameters = {"json", module_yaml};
    mc_rtc::log::info("Disconnection done, result module in: {}", module_yaml);
  }

  return out;
}

} // namespace mc_rbdyn
