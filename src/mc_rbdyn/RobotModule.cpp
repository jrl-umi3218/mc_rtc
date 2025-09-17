/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/config.h>
#include <mc_rtc/path.h>

#include <mc_rbdyn/Robot.h>

#include <mesh_sampling/mesh_sampling.h>

namespace fs = std::filesystem;

namespace mc_rbdyn
{

// Repeat static constexpr declarations
// See also https://stackoverflow.com/q/8016780
constexpr double RobotModule::Gripper::Safety::DEFAULT_PERCENT_VMAX;
constexpr double RobotModule::Gripper::Safety::DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER;
constexpr double RobotModule::Gripper::Safety::DEFAULT_RELEASE_OFFSET;
constexpr unsigned int RobotModule::Gripper::Safety::DEFAULT_OVER_COMMAND_LIMIT_ITER_N;

DevicePtrVector::DevicePtrVector(const DevicePtrVector & v) : std::vector<DevicePtr>()
{
  reserve(v.size());
  for(const auto & s : v) { push_back(s->clone()); }
}

DevicePtrVector & DevicePtrVector::operator=(const DevicePtrVector & v)
{
  if(&v == this) { return *this; }
  resize(v.size());
  for(size_t i = 0; i < v.size(); ++i) { (*this)[i] = v[i]->clone(); }
  return *this;
}

RobotModule::RobotModule(const std::string & name, const rbd::parsers::ParserResult & res)
: RobotModule("/CREATED/BY/MC/RTC/", name)
{
  rsdf_dir = "";
  convex_dir = "";
  init(res);
}

void RobotModule::init(const rbd::parsers::ParserResult & res)
{
  mb = res.mb;
  mbc = res.mbc;
  mbg = res.mbg;
  for(const auto & col : res.collision)
  {
    const auto & body = col.first;
    const auto & cols = col.second;
    if(cols.size()) { _collisionTransforms[body] = cols[0].origin; }
  }
  boundsFromURDF(res.limits);
  _visual = res.visual;
  _collision = res.collision;
  if(_ref_joint_order.size() == 0) { make_default_ref_joint_order(); }
  expand_stance();
  generate_convexes();
  bind_convexes();
}

void RobotModule::generate_convexes(bool regenerate, unsigned int sampling_point)
{
  auto urdfCRC = [&]()
  {
    uint32_t crc = 0;
    for(const auto & col : _collision)
    {
      for(const auto & c : col.second)
      {
        if(c.geometry.type == rbd::parsers::Geometry::Type::MESH)
        {
          const auto & mesh_path = fs::path(boost::get<rbd::parsers::Geometry::Mesh>(c.geometry.data).filename);
          crc += fs::file_size(mc_rtc::convertURI(mesh_path));
        }
      }
    }
    return crc;
  };

  auto generate = [&](const fs::path & mesh_file, const fs::path & output_convex_dir)
  {
    if(!fs::exists(mesh_file)) { mc_rtc::log::error_and_throw("Couldn't find meshes at {} for {}", mesh_file, name); }

    mesh_sampling::MeshSampling sampler(mesh_file);
    const auto & meshes = sampler.create_clouds(sampling_point);
    sampler.create_convexes(meshes, output_convex_dir, false);
  };

  if(!name.empty())
  {
    auto cache_root = fs::path(mc_rtc::local_share_directory(name));
    fs::create_directories(cache_root);

    std::string variant_name = fs::path(urdf_path).filename().stem().string();
    std::string target_folder_name = variant_name + "-" + std::to_string(urdfCRC());
    convex_dir = cache_root / target_folder_name;

    if(!fs::exists(convex_dir) || regenerate)
    {
      // Clean up any old folders for the same variant
      for(const auto & cache_entry : fs::directory_iterator(cache_root))
      {
        if(!fs::is_directory(cache_entry)) continue;

        std::string folder_name = mc_rtc::filename(cache_entry, true);
        if(folder_name.rfind(variant_name + "-", 0) == 0 && folder_name != target_folder_name)
        {
          fs::remove_all(cache_entry);
        }
      }

      fs::create_directories(convex_dir);

      for(const auto & col : _collision)
      {
        for(const auto & c : col.second)
        {
          if(c.geometry.type == rbd::parsers::Geometry::Type::MESH)
          {
            const auto & mesh_file = fs::path(boost::get<rbd::parsers::Geometry::Mesh>(c.geometry.data).filename);
            generate(mc_rtc::convertURI(mesh_file), convex_dir);
          }
        }
      }
    }
  }
  else { mc_rtc::log::error("[RobotModule] name is empty, cannot generate convexes"); }
}

void RobotModule::bind_convexes()
{
  for(const auto & b : mb.bodies())
  {
    const auto & collisions = _collision[b.name()];
    if(collisions.size() == 1)
    {
      if(collisions[0].geometry.type == rbd::parsers::Geometry::Type::MESH)
      {
        const auto & mesh_file = boost::get<rbd::parsers::Geometry::Mesh>(collisions[0].geometry.data).filename;
        fs::path convex_path = fs::path(convex_dir) / (mc_rtc::filename(mesh_file) + "-ch.txt");

        if(!fs::exists(convex_path))
        {
          mc_rtc::log::error("Convex hull file does not exist: {}", convex_path.string());
          continue;
        }

        _convexHull[b.name()] = {b.name(), convex_path};
      }
      continue;
    }

    size_t added = 0;
    for(const auto & col : collisions)
    {
      if(col.geometry.type == rbd::parsers::Geometry::Type::MESH)
      {
        const auto & mesh_file = boost::get<rbd::parsers::Geometry::Mesh>(col.geometry.data).filename;
        fs::path convex_path = fs::path(convex_dir) / (mc_rtc::filename(mesh_file) + "-ch.txt");

        if(!fs::exists(convex_path))
        {
          mc_rtc::log::error("Convex hull file does not exist: {}", convex_path.string());
          continue;
        }

        _convexHull[b.name() + "_" + std::to_string(added)] = {b.name() + "_" + std::to_string(added), convex_path};
        added++;
      }
    }
  }
}

RobotModule::Gripper::Gripper(const std::string & name, const std::vector<std::string> & joints, bool reverse_limits)
: Gripper(name, joints, reverse_limits, nullptr, nullptr)
{
}

RobotModule::Gripper::Gripper(const std::string & name,
                              const std::vector<std::string> & joints,
                              bool reverse_limits,
                              const Safety & safety)
: Gripper(name, joints, reverse_limits, &safety, nullptr)
{
}

RobotModule::Gripper::Gripper(const std::string & name,
                              const std::vector<std::string> & joints,
                              bool reverse_limits,
                              const Safety & safety,
                              const std::vector<Mimic> & mimics)
: Gripper(name, joints, reverse_limits, &safety, &mimics)
{
}

RobotModule::Gripper::Gripper(const std::string & name,
                              const std::vector<std::string> & joints,
                              bool reverse_limits,
                              const Safety * safety,
                              const std::vector<Mimic> * mimics)
: name(name), joints(joints), reverse_limits(reverse_limits), hasSafety_(safety != nullptr),
  hasMimics_(mimics != nullptr)
{
  if(mimics) { mimics_ = *mimics; }
  if(safety) { safety_ = *safety; }
}

void RobotModule::Gripper::Safety::load(const mc_rtc::Configuration & config)
{
  if(config.has("actualCommandDiffTrigger")) { actualCommandDiffTrigger = config("actualCommandDiffTrigger"); }
  if(config.has("threshold"))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED] Gripper safety property \"threshold\" (expressed in degrees) is "
                         "deprecated, please use \"actualCommandDiffTrigger\" (expressed in radian) instead");
    actualCommandDiffTrigger = mc_rtc::constants::toRad(config("threshold"));
  }

  if(config.has("overCommandLimitIterN"))
  {
    overCommandLimitIterN =
        std::max<unsigned int>(1, config("overCommandLimitIterN", DEFAULT_OVER_COMMAND_LIMIT_ITER_N));
  }
  else if(config.has("iter"))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED] Gripper safety property \"iter\" is deprecated, please use "
                         "\"overCommandLimitIterN\" instead");
    overCommandLimitIterN = std::max<unsigned int>(1, config("iter", DEFAULT_OVER_COMMAND_LIMIT_ITER_N));
  }

  if(config.has("releaseSafetyOffset")) { releaseSafetyOffset = config("releaseSafetyOffset"); }
  else if(config.has("release"))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED] Gripper safety property \"release\" (expressed in degrees) is "
                         "deprecated, please use \"releaseSafetyOffset\" instead (expressed in radian)");
    releaseSafetyOffset = mc_rtc::constants::toRad(config("release"));
  }

  if(config.has("percentVMax"))
  {
    percentVMax = mc_filter::utils::clamp(static_cast<double>(config("percentVMax")), 0, 1);
  }
  else if(config.has("percentVMAX"))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED] Gripper safety property \"percentVMAX\" is deprecated, please use "
                         "\"percentVMax\" instead");
    percentVMax = mc_filter::utils::clamp(static_cast<double>(config("percentVMAX")), 0, 1);
  }
}

mc_rtc::Configuration RobotModule::Gripper::Safety::save() const
{
  mc_rtc::Configuration config;
  config.add("actualCommandDiffTrigger", actualCommandDiffTrigger);
  config.add("overCommandLimitIterN", overCommandLimitIterN);
  config.add("releaseSafetyOffset", releaseSafetyOffset);
  config.add("percentVMax", percentVMax);
  return config;
}

void RobotModule::boundsFromURDF(const rbd::parsers::Limits & limits)
{
  _bounds = urdf_limits_to_bounds(limits);
}

void RobotModule::expand_stance()
{
  for(const auto & j : mb.joints())
  {
    if(!_stance.count(j.name()) && j.name() != "Root") { _stance[j.name()] = j.zeroParam(); }
  }
}

void RobotModule::make_default_ref_joint_order()
{
  _ref_joint_order.resize(0);
  for(const auto & j : mb.joints())
  {
    if(j.dof() >= 1 && j.type() != rbd::Joint::Free) { _ref_joint_order.push_back(j.name()); }
  }
}

RobotModule::bounds_t urdf_limits_to_bounds(const rbd::parsers::Limits & limits)
{
  RobotModule::bounds_t ret = {};
  ret.reserve(6);
  ret.push_back(limits.lower);
  ret.push_back(limits.upper);
  auto convert = [](const std::map<std::string, std::vector<double>> & l)
  {
    auto ret = l;
    for(auto & el : ret)
    {
      for(auto & e : el.second) { e = -e; }
    }
    return ret;
  };
  ret.push_back(convert(limits.velocity));
  ret.push_back(limits.velocity);
  ret.push_back(convert(limits.torque));
  ret.push_back(limits.torque);
  return ret;
}

bool check_module_compatibility(const RobotModule & lhs, const RobotModule & rhs)
{
  bool is_ok = true;
  auto format_params = [](const std::vector<std::string> & params) -> std::string
  {
    if(params.size() == 0) { return "[]"; }
    std::stringstream ss;
    ss << "[";
    for(size_t i = 0; i < params.size() - 1; ++i) { ss << params[i] << ", "; }
    ss << params.back() << "]";
    return ss.str();
  };
  auto show_incompatiblity_issue = [&](std::string_view reason)
  {
    if(is_ok)
    {
      is_ok = false;
      mc_rtc::log::critical("{} (params: {}) and {} (params: {}) are incompatible.\nReasons:", lhs.name,
                            format_params(lhs.parameters()), rhs.name, format_params(rhs.parameters()));
    }
    mc_rtc::log::critical("- {}", reason);
  };
  if(lhs.ref_joint_order() != rhs.ref_joint_order()) { show_incompatiblity_issue("Different reference joint order"); }
  if(lhs.bodySensors() != rhs.bodySensors()) { show_incompatiblity_issue("Different body sensors"); }
  if(lhs.forceSensors() != rhs.forceSensors()) { show_incompatiblity_issue("Different force sensors"); }
  if(lhs.jointSensors() != rhs.jointSensors()) { show_incompatiblity_issue("Different joint sensors"); }
  if(lhs.grippers() != rhs.grippers()) { show_incompatiblity_issue("Different grippers"); }
  if(lhs.devices().size() != rhs.devices().size()) { show_incompatiblity_issue("Different devices"); }
  else
  {
    const auto & lhs_devices = lhs.devices();
    const auto & rhs_devices = rhs.devices();
    for(size_t i = 0; i < lhs_devices.size(); ++i)
    {
      const auto & lhs_d = *lhs_devices[i];
      const auto & rhs_d = *rhs_devices[i];
      if(lhs_d.name() != rhs_d.name() || lhs_d.type() != rhs_d.type() || lhs_d.parent() != rhs_d.parent())
      {
        show_incompatiblity_issue(fmt::format("Different device: {} != {}", lhs_d.name(), rhs_d.name()));
      }
    }
  }
  return is_ok;
}

} // namespace mc_rbdyn
