#ifndef _H_STANCECONFIG_H_
#define _H_STANCECONFIG_H_

/* This struct holds the configuration for stances in a seq plan */

/* TODO Fill-it up */
/* TODO Build a tool to import from python */
/* TODO Build a tool to read from configuration file */

#include <Eigen/Core>
#include <SpaceVecAlg/SpaceVecAlg>
#include <functional>

namespace mc_rbdyn
{

struct StanceConfig
{
public:
  struct CoMTask
  {
    double stiffness;
    double extraStiffness;
    double weight;
  };
  struct CoMObj
  {
    Eigen::Vector3d comOffset;
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
    double thresh;
    std::function<Eigen::Vector3d (const sva::PTransformd &, const sva::PTransformd &, const Eigen::Vector3d &)> pos;
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
    Eigen::Vector3d adjustOriTBNWeight;
    double preContactDist;
  };
public:
  CoMTask comTask;
  CoMObj comObj;
  PostureTask postureTask;
  ContactTask contactTask;
  ContactObj contactObj;
};

}

#endif
