#include <mc_rbdyn/BodySensor.h>

namespace mc_rbdyn
{

BodySensor::~BodySensor() = default;

BodySensor * BodySensor::clone() const
{
  return new BodySensor(*this);
}

} // namespace mc_rbdyn
