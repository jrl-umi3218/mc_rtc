#include <mc_rbdyn/JointSensor.h>

namespace mc_rbdyn
{

JointSensor::JointSensor(const std::string & jointName) : JointSensor("js-" + jointName, jointName) {}

} // namespace mc_rbdyn
