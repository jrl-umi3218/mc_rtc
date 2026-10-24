/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>
#include <mc_rtc/config.h>
#include <mc_rtc/path.h>

// For CRC32
#include <cstddef>
#include <cstdint>
#include <fstream> // IWYU pragma: keep

#include <mc_rbdyn/Robot.h>

#include <mesh_sampling/mesh_sampling.h>

namespace fs = std::filesystem;

namespace mc_rbdyn
{

namespace details
{
/**
 * Simple CRC32 implementation to compute a CRC32 checksum of a file. This is used to determine if the convex hulls need
 * to be regenerated. It is based on the well-known bitwise CRC-32 algorithm using the polynomial `0xEDB88320`, which is
 * the reversed form of the standard CRC-32 polynomial. This implementation is not copied from any specific source, but
 * follows the canonical algorithm described in many references, such as:
 *
 * - [Wikipedia: CRC-32 algorithm](https://en.wikipedia.org/wiki/Cyclic_redundancy_check#CRC-32_algorithm)
 * - [Ross Williams' "A Painless Guide to CRC Error Detection Algorithms"](https://zlib.net/crc_v3.txt)
 * - The reference implementation in [zlib](https://github.com/madler/zlib/blob/master/crc32.c) (though zlib uses a
 * table-based approach for speed)
 */
struct CRC32
{
  CRC32() : crc_(0xFFFFFFFF) {}

  void update(const void * data, size_t length)
  {
    const uint8_t * buf = static_cast<const uint8_t *>(data);
    for(size_t i = 0; i < length; ++i)
    {
      crc_ ^= buf[i];
      for(int j = 0; j < 8; ++j)
      {
        if(crc_ & 1)
          crc_ = (crc_ >> 1) ^ 0xEDB88320;
        else
          crc_ >>= 1;
      }
    }
  }

  uint32_t finalize() const { return crc_ ^ 0xFFFFFFFF; }

private:
  uint32_t crc_;
};
} // namespace details

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
  // FIXME revert this once https://github.com/jrl-umi3218/mesh_sampling/pull/6 has been resolved.
  const char * disableConvexGen = std::getenv("MC_RTC_DISABLE_CONVEX_GENERATION_PATCH");
  if(!(disableConvexGen && std::string(disableConvexGen) == "ON"))
  {
    generate_convexes();
    bind_convexes();
  }
}

/**
 * Name of the convex hull file of a collision mesh. A scaled mesh has its scale in the name, so that a mesh used both
 * as is and mirrored gets two hulls
 */
std::string convexName(const rbd::parsers::Geometry::Mesh & mesh)
{
  auto name = mc_rtc::filename(mesh.filename);
  if(!mesh.scaleV.isOnes()) { name += fmt::format("_{}_{}_{}", mesh.scaleV.x(), mesh.scaleV.y(), mesh.scaleV.z()); }
  return name + "-ch.txt";
}

void RobotModule::generate_convexes(bool regenerate, unsigned int sampling_point)
{
  auto urdfCRC = [&]()
  {
    mc_rbdyn::details::CRC32 crc;
    for(const auto & col : _collision)
    {
      for(const auto & c : col.second)
      {
        if(c.geometry.type == rbd::parsers::Geometry::Type::MESH)
        {
          const auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(c.geometry.data);
          // Read file contents
          std::ifstream file(mc_rtc::convertURI(fs::path(mesh.filename)), std::ios::binary);
          if(file)
          {
            std::vector<char> buffer((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            if(!buffer.empty()) crc.update(buffer.data(), buffer.size());
          }
          // Add scale values
          for(Eigen::Index i = 0; i < 3; ++i)
          {
            double scale = mesh.scaleV[i];
            crc.update(&scale, sizeof(double));
          }
        }
      }
    }
    return crc.finalize();
  };

  auto generate = [&](const rbd::parsers::Geometry::Mesh & mesh, const fs::path & output_convex_dir)
  {
    const auto mesh_file = mc_rtc::convertURI(fs::path(mesh.filename));
    if(!fs::exists(mesh_file))
    {
      mc_rtc::log::error_and_throw("Couldn't find meshes at {} for {}", mesh_file.string(), name);
    }

    try
    {
      mesh_sampling::MeshSampling sampler(mesh_file, mesh.scaleV.cast<float>());
      // The scale of the mesh in the robot description, e.g. 0.001 for a mesh in millimeters, -1 on an axis to mirror it
      auto cloud = sampler.create_cloud(mesh_file, sampling_point, {}, false);
      sampler.create_convex(cloud, output_convex_dir / convexName(mesh));
    }
    catch(const std::exception & e)
    {
      mc_rtc::log::error("[RobotModule] {}: could not generate the convex hull of {}: {}", name, mesh_file.string(),
                         e.what());
    }
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
            generate(boost::get<rbd::parsers::Geometry::Mesh>(c.geometry.data), convex_dir);
          }
        }
      }
    }
  }
  else
  {
    mc_rtc::log::error("[RobotModule] name is empty, cannot generate convexes");
  }
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
        const auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(collisions[0].geometry.data);
        fs::path convex_path = fs::path(convex_dir) / convexName(mesh);

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
        const auto & mesh = boost::get<rbd::parsers::Geometry::Mesh>(col.geometry.data);
        fs::path convex_path = fs::path(convex_dir) / convexName(mesh);

        if(!fs::exists(convex_path))
        {
          mc_rtc::log::error("Convex hull file does not exist: {}", convex_path.string());
          continue;
        }

        _convexHull[b.name() + "_" + std::to_string(added)] = {b.name(), convex_path};
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
