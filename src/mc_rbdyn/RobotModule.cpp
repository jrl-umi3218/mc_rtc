#include <mc_rbdyn/RobotModule.h>

namespace mc_rbdyn
{

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

void RobotModule::boundsFromURDF(const mc_rbdyn_urdf::Limits & limits)
{
  auto neg_bound = [](const std::map<std::string, std::vector<double>> & v)
  {
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
    limits.lower,
    limits.upper,
    neg_bound(limits.velocity),
    limits.velocity,
    neg_bound(limits.torque),
    limits.torque,
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
    if(j.dof() >= 1 && j.type() == rbd::Joint::Free)
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
  auto convert = [](const std::map<std::string, std::vector<double>> & l)
  {
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

}
