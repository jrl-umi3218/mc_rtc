#include <mc_control/ForceContactSensor.h>

#include <mc_rbdyn/Surface.h>

namespace mc_control
{

Threshold::Threshold(double t, unsigned int i)
: thresh(t), iter(i)
{
}

ForceSensor::ForceSensor(const mc_rbdyn::Robot & robot, const std::string & sensorName, const Threshold & thresh)
: activated(Unactivated), thresh(thresh), activatedIter(0), direction(Unactivated),
  lastValues(WindowSize), offset(0), name(sensorName), surfacesName(0)
{
  std::string parent = robot.forceSensor(sensorName).parentBody();
  for(const auto & p : robot.surfaces())
  {
    if(p.second->bodyName() == parent)
    {
      surfacesName.push_back(p.first);
    }
  }
}

void ForceSensor::computeOffset()
{
  double off = 0;
  for(const auto & v : lastValues)
  {
    off += v;
  }
  off = off / static_cast<double>(lastValues.size());
  offset = off;
}

void ForceSensor::update(const Eigen::Vector3d & force)
{
  lastValues.push_back(force.z());
  double forceZ = force.z() - offset;
  if(std::abs(forceZ) > thresh.thresh)
  {
    activatedIter++;
    if(forceZ >= 0)
    {
      direction = Forward;
      if(activatedIter > thresh.iter)
      {
        activated = Activated;
      }
    }
    else
    {
      direction = Backward;
      if(activatedIter > thresh.iter + 5)
      {
        activated = Activated;
      }
    }
  }
  else
  {
    activatedIter = 0;
    activated = Unactivated;
  }
}

ForceContactSensor::ForceContactSensor(const mc_rbdyn::Robot & robot)
{
  /*FIXME Hard-coded */
  sensors.push_back(ForceSensor(robot, "RightFootForceSensor", Threshold(200, 9)));
  sensors.push_back(ForceSensor(robot, "LeftFootForceSensor", Threshold(200, 9)));
  sensors.push_back(ForceSensor(robot, "RightHandForceSensor", Threshold(5, 3)));
  sensors.push_back(ForceSensor(robot, "LeftHandForceSensor", Threshold(5, 3)));
}

void ForceContactSensor::resetOffset()
{
  for(auto & s : sensors)
  {
    s.computeOffset();
  }
}

std::vector<std::string> ForceContactSensor::update(MCController & ctl)
{
  std::vector<std::string> res;

  for(auto& sensor: sensors)
  {
    if(ctl.robot().hasForceSensor(sensor.name))
    {
      sensor.update(ctl.robot().forceSensor(sensor.name).wrench().force());
    }
    if(sensor.activated == ForceSensor::Activated)
    {
      if(sensor.direction == ForceSensor::Forward)
      {
        for(const auto & sn : sensor.surfacesName)
        {
          res.push_back(sn);
        }
      }
      else
      {
        for(const auto & sn : sensor.surfacesName)
        {
          res.push_back(sn + "_back");
        }
      }
    }
  }
  return res;
}

}
