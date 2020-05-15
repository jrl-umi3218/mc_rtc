#include <mc_rbdyn/BodySensor.h>

namespace mc_rbdyn
{

BodySensor::~BodySensor() noexcept = default;

DevicePtr BodySensor::clone() const
{
  return DevicePtr(new BodySensor(*this));
}

} // namespace mc_rbdyn
