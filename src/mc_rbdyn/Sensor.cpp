#include <mc_rbdyn/Sensor.h>

#include <mc_rbdyn/Robot.h>

namespace mc_rbdyn
{

Sensor::Sensor(const std::string & name, const std::string & parent, const sva::PTransformd & X_p_s)
: type_(""), name_(name), parent_(parent), X_p_s_(X_p_s)
{
}

sva::PTransformd Sensor::X_0_s(const mc_rbdyn::Robot & robot) const
{
  return X_p_s() * robot.bodyPosW(parent_);
}

} // namespace mc_rbdyn
