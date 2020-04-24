#include <mc_rbdyn/BodySensor.h>

namespace mc_rbdyn
{

BodySensor::~BodySensor() noexcept = default;

SensorPtr BodySensor::clone() const
{
  return SensorPtr(new BodySensor(*this));
}

} // namespace mc_rbdyn
