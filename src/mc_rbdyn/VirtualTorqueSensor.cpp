#include <mc_rbdyn/VirtualTorqueSensor.h>

namespace mc_rbdyn
{

VirtualTorqueSensor::VirtualTorqueSensor() : VirtualTorqueSensor("", 0) {}

VirtualTorqueSensor::VirtualTorqueSensor(const std::string & name, const int & size)
: Device(name), virtualJointTorques_(Eigen::VectorXd::Zero(size)), size_(size)
{
}

VirtualTorqueSensor::VirtualTorqueSensor(const VirtualTorqueSensor & ets) : VirtualTorqueSensor(ets.name_, ets.size_)
{
  virtualJointTorques_ = ets.virtualJointTorques_;
}

VirtualTorqueSensor & VirtualTorqueSensor::operator=(const VirtualTorqueSensor & ets)
{
  if(&ets == this) { return *this; }
  name_ = ets.name_;
  parent_ = ets.parent_;
  X_p_s_ = ets.X_p_s_;
  virtualJointTorques_ = ets.virtualJointTorques_;
  return *this;
}

VirtualTorqueSensor::~VirtualTorqueSensor() noexcept = default;

DevicePtr VirtualTorqueSensor::clone() const
{
  return DevicePtr(new VirtualTorqueSensor(*this));
}

} // namespace mc_rbdyn
