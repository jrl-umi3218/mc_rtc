/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>

namespace mc_rbdyn
{

// Repeat static constexpr declarations
// See also https://stackoverflow.com/q/8016780
constexpr double RobotModule::Gripper::Safety::DEFAULT_PERCENT_VMAX;
constexpr double RobotModule::Gripper::Safety::DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER;
constexpr double RobotModule::Gripper::Safety::DEFAULT_RELEASE_OFFSET;
constexpr unsigned int RobotModule::Gripper::Safety::DEFAULT_OVER_COMMAND_LIMIT_ITER_N;

SensorPtrVector::SensorPtrVector(const SensorPtrVector & v) : std::vector<SensorPtr>()
{
  reserve(v.size());
  for(const auto & s : v)
  {
    push_back(s->clone());
  }
}

SensorPtrVector & SensorPtrVector::operator=(const SensorPtrVector & v)
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

RobotModule::RobotModule(const std::string & name, const mc_rbdyn_urdf::URDFParserResult & res)
: RobotModule("/CREATED/BY/MC/RTC/", name)
{
  rsdf_dir = "";
  mb = res.mb;
  mbc = res.mbc;
  mbg = res.mbg;
  _collisionTransforms = res.collision_tf;
  boundsFromURDF(res.limits);
  _visual = res.visual;
  make_default_ref_joint_order();
  expand_stance();
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

void RobotModule::boundsFromURDF(const mc_rbdyn_urdf::Limits & limits)
{
  auto neg_bound = [](const std::map<std::string, std::vector<double>> & v) {
    std::map<std::string, std::vector<double>> res;
    for(const auto & vi : v)
    {
      res[vi.first] = vi.second;
      for(auto & vj : res[vi.first])
      {
        vj = -vj;
      }
    }
    return res;
  };
  _bounds = {
      limits.lower, limits.upper, neg_bound(limits.velocity), limits.velocity, neg_bound(limits.torque), limits.torque,
  };
}

void RobotModule::expand_stance()
{
  for(const auto & j : mb.joints())
  {
    if(!_stance.count(j.name()))
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

RobotModule::bounds_t urdf_limits_to_bounds(const mc_rbdyn_urdf::Limits & limits)
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
