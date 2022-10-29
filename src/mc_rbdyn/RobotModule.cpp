/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>

#include <mc_rbdyn/Robot.h>

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
  for(const auto & s : v)
  {
    push_back(s->clone());
  }
}

DevicePtrVector & DevicePtrVector::operator=(const DevicePtrVector & v)
{
  if(&v == this)
  {
    return *this;
  }
  resize(v.size());
  for(size_t i = 0; i < v.size(); ++i)
  {
    (*this)[i] = v[i]->clone();
  }
  return *this;
}

RobotModule::RobotModule(const std::string & name, const rbd::parsers::ParserResult & res)
: RobotModule("/CREATED/BY/MC/RTC/", name)
{
  rsdf_dir = "";
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
    if(cols.size())
    {
      _collisionTransforms[body] = cols[0].origin;
    }
  }
  boundsFromURDF(res.limits);
  _visual = res.visual;
  _collision = res.collision;
  if(_ref_joint_order.size() == 0)
  {
    make_default_ref_joint_order();
  }
  expand_stance();
  setupDefaultControlToCanonical();
}

void RobotModule::setupDefaultControlToCanonical()
{
  // Implement default control to canonical function
  bool first = true;
  // Common joint indices from control robot -> canonical robot
  auto commonJointIndices = std::vector<std::pair<unsigned int, unsigned int>>{};
  // Common encoder indices from control robot -> canonical robot
  auto commonEncoderIndices = std::vector<std::pair<unsigned int, unsigned int>>{};
  // Indices of joints with mimics from actuated joints in control robot
  // to their mimic counterpart in canonical robot
  auto mimicJoints = std::vector<std::pair<unsigned int, unsigned int>>{};

  defaultControlToCanonical = [first, commonJointIndices, commonEncoderIndices,
                               mimicJoints](const mc_rbdyn::Robot & control, mc_rbdyn::Robot & canonical) mutable {
    // The first time this function is ran, initialize the fixed list of common joints
    // between the control robot and canonical robot
    if(first)
    {
      // Contruct list of common joints between common and canonical robots
      commonJointIndices.reserve(std::max(control.mb().joints().size(), canonical.mb().joints().size()));
      for(const auto & joint : control.mb().joints())
      {
        const auto & jname = joint.name();
        if(canonical.hasJoint(jname)
           && canonical.mb().joint(static_cast<int>(canonical.jointIndexByName(jname))).dof() == joint.dof())
        {
          commonJointIndices.emplace_back(control.jointIndexByName(jname), canonical.jointIndexByName(jname));
        }
      }

      // Construct the list of common actuated joints between the
      // control and canonical robot
      const auto & rjo = control.refJointOrder();
      for(size_t i = 0; i < rjo.size(); ++i)
      {
        auto mbcIndex = canonical.jointIndexInMBC(i);
        if(mbcIndex < 0)
        {
          continue;
        }
        commonEncoderIndices.emplace_back(i, mbcIndex);
      }

      for(const auto & m : canonical.mb().joints())
      {
        if(m.isMimic())
        {
          auto mainIndex = canonical.jointIndexByName(m.mimicName());
          auto mimicIndex = canonical.jointIndexByName(m.name());
          mimicJoints.emplace_back(mainIndex, mimicIndex);
        }
      }
      first = false;
    }

    // Copy the common mbc joints into canonical
    for(const auto & commonIndices : commonJointIndices)
    {
      canonical.mbc().q[commonIndices.second] = control.mbc().q[commonIndices.first];
    }

    // Handle mimics in canonical
    for(const auto & mimicIndices : mimicJoints)
    {
      const auto & m = canonical.mb().joint(static_cast<int>(mimicIndices.second));
      canonical.mbc().q[mimicIndices.second][0] =
          m.mimicMultiplier() * canonical.mbc().q[mimicIndices.first][0] + m.mimicOffset();
    }

    // Copy the encoders into canonical
    const auto & encoders = control.encoderValues();
    if(encoders.size() == control.refJointOrder().size())
    {
      for(const auto & indices : commonEncoderIndices)
      {
        canonical.mbc().q[indices.second][0] = encoders[indices.first];
      }
    }

    // Copy the force sensors into canonical
    for(const auto & fs : control.forceSensors())
    {
      if(canonical.hasForceSensor(fs.name()))
      {
        canonical.forceSensor(fs.name()) = fs;
      }
    }

    // Copy the body sensors into canonical
    for(const auto & bs : control.bodySensors())
    {
      canonical.bodySensor(bs.name()) = bs;
    }

    // Copy the base position which triggers all the updates
    canonical.posW(control.posW());
  };
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
  if(mimics)
  {
    mimics_ = *mimics;
  }
  if(safety)
  {
    safety_ = *safety;
  }
}

void RobotModule::Gripper::Safety::load(const mc_rtc::Configuration & config)
{
  if(config.has("actualCommandDiffTrigger"))
  {
    actualCommandDiffTrigger = config("actualCommandDiffTrigger");
  }
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

  if(config.has("releaseSafetyOffset"))
  {
    releaseSafetyOffset = config("releaseSafetyOffset");
  }
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
    if(!_stance.count(j.name()) && j.name() != "Root")
    {
      _stance[j.name()] = j.zeroParam();
    }
  }
}

void RobotModule::make_default_ref_joint_order()
{
  _ref_joint_order.resize(0);
  for(const auto & j : mb.joints())
  {
    if(j.dof() >= 1 && j.type() != rbd::Joint::Free)
    {
      _ref_joint_order.push_back(j.name());
    }
  }
}

RobotModule::bounds_t urdf_limits_to_bounds(const rbd::parsers::Limits & limits)
{
  RobotModule::bounds_t ret = {};
  ret.reserve(6);
  ret.push_back(limits.lower);
  ret.push_back(limits.upper);
  auto convert = [](const std::map<std::string, std::vector<double>> & l) {
    auto ret = l;
    for(auto & el : ret)
    {
      for(auto & e : el.second)
      {
        e = -e;
      }
    }
    return ret;
  };
  ret.push_back(convert(limits.velocity));
  ret.push_back(limits.velocity);
  ret.push_back(convert(limits.torque));
  ret.push_back(limits.torque);
  return ret;
}

} // namespace mc_rbdyn
