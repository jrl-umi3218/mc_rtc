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
  lastValues(WindowSize), offset(0), surfacesName(0)
{
  std::string parent = robot.forceSensorParentBodyName(sensorName);
  for(const auto & p : robot.surfaces)
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
  off = off / lastValues.size();
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
  sensors.push_back(ForceSensor(robot, "RightFootForceSensor", Threshold(150, 9)));
  sensors.push_back(ForceSensor(robot, "LeftFootForceSensor", Threshold(150, 9)));
  sensors.push_back(ForceSensor(robot, "RightHandForceSensor", Threshold(25, 1)));
  sensors.push_back(ForceSensor(robot, "LeftHandForceSensor", Threshold(25, 1)));
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

  for(size_t i = 0; i < sensors.size(); ++i)
  {
    sensors[i].update(ctl.wrenches[i].first);
    if(sensors[i].activated == ForceSensor::Activated)
    {
      if(sensors[i].direction == ForceSensor::Forward)
      {
        for(const auto & sn : sensors[i].surfacesName)
        {
          res.push_back(sn);
        }
      }
      else
      {
        for(const auto & sn : sensors[i].surfacesName)
        {
          res.push_back(sn + "_back");
        }
      }
    }
  }

  return res;
}

}
