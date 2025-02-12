#include <mc_rbdyn/ExternalTorqueSensor.h>

namespace mc_rbdyn
{

ExternalTorqueSensor::ExternalTorqueSensor() : ExternalTorqueSensor("", 0) {}

ExternalTorqueSensor::ExternalTorqueSensor(const std::string & name, const int & size)
: Device(name), externalJointTorques_(Eigen::VectorXd::Zero(size)), size_(size)
{
}

ExternalTorqueSensor::ExternalTorqueSensor(const ExternalTorqueSensor & ets)
: ExternalTorqueSensor(ets.name_, ets.size_)
{
  externalJointTorques_ = ets.externalJointTorques_;
}

ExternalTorqueSensor & ExternalTorqueSensor::operator=(const ExternalTorqueSensor & ets)
{
  if(&ets == this) { return *this; }
  name_ = ets.name_;
  parent_ = ets.parent_;
  X_p_s_ = ets.X_p_s_;
  externalJointTorques_ = ets.externalJointTorques_;
  return *this;
}

ExternalTorqueSensor::~ExternalTorqueSensor() noexcept = default;

DevicePtr ExternalTorqueSensor::clone() const
{
  return DevicePtr(new ExternalTorqueSensor(*this));
}

} // namespace mc_rbdyn
