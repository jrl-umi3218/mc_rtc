#include <mc_rbdyn/StanceConfig.h>

namespace mc_rbdyn
{

std::function<Eigen::Vector3d (const sva::PTransformd &, const sva::PTransformd &, const Eigen::Vector3d &)>
percentWaypoint(double x, double y, double z, double nOff)
{
  return [x, y, z, nOff](const sva::PTransformd & start, const sva::PTransformd & end, const Eigen::Vector3d & N)
  {
    Eigen::Vector3d dist = end.translation() - start.translation();
    return start.translation() + Eigen::Vector3d(dist.x()*x, dist.y()*y, dist.z()*z) + N*nOff;
  };
}

std::function<Eigen::Vector3d (const sva::PTransformd &, const sva::PTransformd &, const Eigen::Vector3d &)>
hardCodedPos(double x, double y, double z)
{
  return [x, y, z](const sva::PTransformd &, const sva::PTransformd &, const Eigen::Vector3d &)
  {
    return Eigen::Vector3d(x,y,z);
  };
}

StanceConfig::StanceConfig()
{
  comTask.stiffness = 5.0;
  comTask.extraStiffness = 0.0;
  comTask.weight = 500.0;
  comTask.targetSpeed = 0.003;

  comObj.posThresh = 0.1;
  comObj.velThresh = 0.000099;
  comObj.comOffset = Eigen::Vector3d(0,0,0);;

  postureTask.stiffness = 2.0;
  postureTask.weight = 10.0;

  contactTask.position.stiffness = 5.0;
  contactTask.position.extraStiffness = 12.0;
  contactTask.position.weight = 300.0;
  contactTask.position.targetSpeed = 0.001;

  contactTask.orientation.stiffness = 15.0;
  contactTask.orientation.weight = 1.0;
  contactTask.orientation.finalWeight = 1000.0;

  contactTask.linVel.stiffness = 10.0;
  contactTask.linVel.weight = 10000.0;
  contactTask.linVel.speed = 0.05;

  contactTask.waypointConf.thresh = 0.15;
  contactTask.waypointConf.pos = percentWaypoint(0.5, 0.5, 0.5, 0.2);

  contactTask.collisionConf.iDist = 0.01;
  contactTask.collisionConf.sDist = 0.005;
  contactTask.collisionConf.damping = 0.05;

  contactObj.posThresh = 0.03;
  contactObj.velThresh = 0.005;
  contactObj.preContactDist = 0.02;
}

}
