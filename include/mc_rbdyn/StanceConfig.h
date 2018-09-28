#ifndef _H_STANCECONFIG_H_
#define _H_STANCECONFIG_H_

/* This struct holds the configuration for stances in a seq plan */

#include <mc_rbdyn/api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>
#include <functional>
#include <map>
#include <vector>

namespace mc_rbdyn
{

typedef std::function<Eigen::Vector3d(const sva::PTransformd &, const sva::PTransformd &, const Eigen::Vector3d &)>
    WaypointFunction;

MC_RBDYN_DLLAPI WaypointFunction
    percentWaypoint(double x, double y, double z, double nOff, double xOff = 0, double yOff = 0, double zOff = 0);

MC_RBDYN_DLLAPI WaypointFunction hardCodedPos(double x, double y, double z);

struct MC_RBDYN_DLLAPI StanceConfig
{
public:
  struct CoMTask
  {
    double stiffness;
    double extraStiffness;
    double weight;
    double targetSpeed;
  };
  struct CoMObj
  {
    double posThresh;
    double velThresh;
    Eigen::Vector3d comOffset;
    Eigen::Vector3d comAdjustOffset;
    double timeout;
  };
  struct PostureTask
  {
    double stiffness;
    double weight;
  };
  struct Position
  {
    double stiffness;
    double extraStiffness;
    double weight;
    double targetSpeed;
  };
  struct Orientation
  {
    double stiffness;
    double weight;
    double finalWeight;
  };
  struct LinVel
  {
    double stiffness;
    double weight;
    double speed;
  };
  struct WaypointConf
  {
    bool skip;
    double thresh;
    WaypointFunction pos;
  };
  struct CollisionConf
  {
    double iDist;
    double sDist;
    double damping;
  };
  struct ContactTask
  {
    Position position;
    Orientation orientation;
    LinVel linVel;
    WaypointConf waypointConf;
    CollisionConf collisionConf;
  };
  struct ContactObj
  {
    double posThresh;
    double velThresh;
    double adjustPosThresh;
    double adjustVelThresh;
    double adjustOriThresh;
    Eigen::Vector3d adjustOffset;
    Eigen::Vector3d adjustRPYOffset;
    Eigen::Vector3d adjustOriTBNWeight;
    double preContactDist;
    double gripperMoveAwayDist;
    bool useComplianceTask;
    double complianceVelThresh;
    Eigen::Vector3d complianceTargetForce;
    Eigen::Vector3d complianceTargetTorque;
  };
  struct BodiesCollisionConf
  {
    std::string body1;
    std::string body2;
    CollisionConf collisionConf;
  };
  struct Collisions
  {
    std::vector<BodiesCollisionConf> autoc;
    std::vector<BodiesCollisionConf> robotEnv;
    std::map<std::pair<std::string, std::string>, std::vector<std::pair<std::string, std::string>>>
        robotEnvContactFilter;
  };

public:
  StanceConfig();

public:
  CoMTask comTask;
  CoMObj comObj;
  PostureTask postureTask;
  ContactTask contactTask;
  ContactObj contactObj;
  Collisions collisions;
};

} // namespace mc_rbdyn

#endif
