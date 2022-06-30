#include <mc_rbdyn/JointSensor.h>

namespace mc_rbdyn
{

JointSensor::~JointSensor() noexcept = default;

DevicePtr JointSensor::clone() const
{
  return DevicePtr(new JointSensor(*this));
}

} // namespace mc_rbdyn
