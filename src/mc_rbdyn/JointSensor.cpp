#include <mc_rbdyn/JointSensor.h>

namespace mc_rbdyn
{

JointSensorPtr JointSensor::clone() const
{
  return JointSensorPtr(new JointSensor(*this));
}

} // namespace mc_rbdyn
