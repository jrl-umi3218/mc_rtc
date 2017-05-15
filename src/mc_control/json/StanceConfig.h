#pragma once

/* A collection of functions for JSON manipulation of
 * StanceConfig objects */

#include <mc_rbdyn/StanceConfig.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <iostream>

namespace mc_rbdyn
{

inline void scCoMTaskFromJSON(StanceConfig::CoMTask & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("extraStiffness", ret.extraStiffness);
  v("weight", ret.weight);
  v("targetSpeed", ret.targetSpeed);
}

inline mc_rtc::Configuration scCoMTaskToJSON(const StanceConfig::CoMTask& ct)
{
  mc_rtc::Configuration conf;
  conf.add("stiffness", ct.stiffness);
  conf.add("extraStiffness", ct.extraStiffness);
  conf.add("weight", ct.weight);
  conf.add("targetSpeed", ct.targetSpeed);
  return conf;
}

inline void scCoMObjFromJSON(StanceConfig::CoMObj & ret, const mc_rtc::Configuration & v)
{
  v("posThresh", ret.posThresh);
  v("velThresh", ret.velThresh);
  v("comOffset", ret.comOffset);
  v("comAdjustOffset", ret.comAdjustOffset);
  v("timeout", ret.timeout);
}

inline mc_rtc::Configuration scCoMObjToJSON(const StanceConfig::CoMObj & co)
{
  mc_rtc::Configuration conf;
  conf.add("posThresh", co.posThresh);
  conf.add("velThresh", co.velThresh);
  conf.add("comOffset", co.comOffset);
  return conf;
}

inline void scPostureTaskFromJSON(StanceConfig::PostureTask & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("weight", ret.weight);
}

inline mc_rtc::Configuration scPostureTaskToJSON(const StanceConfig::PostureTask & pt)
{
  mc_rtc::Configuration conf;
  conf.add("stiffness", pt.stiffness);
  conf.add("weight", pt.weight);
  return conf;
}

inline void scPositionFromJSON(StanceConfig::Position & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("extraStiffness", ret.extraStiffness);
  v("weight", ret.weight);
  v("targetSpeed", ret.targetSpeed);
}

inline mc_rtc::Configuration scPositionToJSON(const StanceConfig::Position& ct)
{
  mc_rtc::Configuration conf;
  conf.add("stiffness", ct.stiffness);
  conf.add("extraStiffness", ct.extraStiffness);
  conf.add("weight", ct.weight);
  conf.add("targetSpeed", ct.targetSpeed);
  return conf;
}

inline void scOrientationFromJSON(StanceConfig::Orientation & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("finalWeight", ret.finalWeight);
  v("weight", ret.weight);
}

inline mc_rtc::Configuration scOrientationToJSON(const StanceConfig::Orientation& ct)
{
  mc_rtc::Configuration conf;
  conf.add("stiffness", ct.stiffness);
  conf.add("weight", ct.weight);
  conf.add("finalWeight", ct.finalWeight);
  return conf;
}

inline void scLinVelFromJSON(StanceConfig::LinVel & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("weight", ret.weight);
  v("speed", ret.speed);
}

inline mc_rtc::Configuration scLinVelToJSON(const StanceConfig::LinVel& ct)
{
  mc_rtc::Configuration conf;
  conf.add("stiffness", ct.stiffness);
  conf.add("weight", ct.weight);
  conf.add("speed", ct.speed);
  return conf;
}

inline void scWaypointConfFromJSON(StanceConfig::WaypointConf & ret, const mc_rtc::Configuration & v)
{
  v("skip", ret.skip);
  v("thresh", ret.thresh);
  if(v.has("type"))
  {
    std::string type = v("type");
    if(type == "percentWaypoint")
    {
      double x = v("conf")("x");
      double y = v("conf")("y");
      double z = v("conf")("z");
      double nOff = v("conf")("nOff");
      double xOff = 0;
      v("conf")("xOff", xOff);
      double yOff = 0;
      v("conf")("yOff", yOff);
      double zOff = 0;
      v("conf")("zOff", zOff);
      ret.pos = percentWaypoint(x, y, z, nOff, xOff, yOff, zOff);
    }
    else if(type == "hardCodedPos")
    {
      double x = v("conf")("x");
      double y = v("conf")("y");
      double z = v("conf")("z");
      ret.pos = hardCodedPos(x, y, z);
    }
    else
    {
      LOG_ERROR("Invalid waypoint type: " << type)
      throw(std::string("Invalid waypoint type in JSON string"));
    }
  }
}

/*FIXME Not very clean and should not be used in practice */
inline mc_rtc::Configuration scWaypointConfToJSON(const StanceConfig::WaypointConf & wpc)
{
  mc_rtc::Configuration conf;
  conf.add("thresh", wpc.thresh);
  /* First check if it is an hardcoded pos */
  sva::PTransformd start = sva::PTransformd::Identity();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  sva::PTransformd pt1 = sva::PTransformd(Eigen::Vector3d(1,1,1));
  sva::PTransformd pt2 = sva::PTransformd(Eigen::Vector3d(2,2,2));
  Eigen::Vector3d pos1 = wpc.pos(start, pt1, N);
  Eigen::Vector3d pos2 = wpc.pos(start, pt2, N);
  if(pos1 == pos2)
  {
    conf.add("type", "hardCodedPos");
    mc_rtc::Configuration wp_conf;
    wp_conf.add("x", pos1.x());
    wp_conf.add("y", pos1.y());
    wp_conf.add("z", pos1.z());
    conf.add("conf", wp_conf);
  }
  else
  {
    conf.add("type", "percentWaypoint");
    mc_rtc::Configuration wp_conf;
    wp_conf.add("x", pos1.x());
    wp_conf.add("y", pos1.y());
    wp_conf.add("z", pos1.z());
    N(0) = 1;
    pos1 = wpc.pos(start, start, N);
    wp_conf.add("nOff", pos1.x());
    conf.add("conf", wp_conf);
  }
  return conf;
}

inline void scCollisionConfFromJSON(StanceConfig::CollisionConf & cc, const mc_rtc::Configuration & v)
{
  v("iDist", cc.iDist);
  v("sDist", cc.sDist);
  v("damping", cc.damping);
}

inline mc_rtc::Configuration scCollisionConfToJSON(const StanceConfig::CollisionConf & cc)
{
  mc_rtc::Configuration conf;
  conf.add("iDist", cc.iDist);
  conf.add("sDist", cc.sDist);
  conf.add("damping", cc.damping);
  return conf;
}

inline void scContactTaskFromJSON(StanceConfig::ContactTask & ct, const mc_rtc::Configuration & v)
{
  if(v.has("position"))
  {
    scPositionFromJSON(ct.position, v("position"));
  }
  if(v.has("orientation"))
  {
     scOrientationFromJSON(ct.orientation, v("orientation"));
  }
  if(v.has("linVel"))
  {
    scLinVelFromJSON(ct.linVel, v("linVel"));
  }
  if(v.has("waypointConf"))
  {
    scWaypointConfFromJSON(ct.waypointConf, v("waypointConf"));
  }
  if(v.has("collisionConf"))
  {
    scCollisionConfFromJSON(ct.collisionConf, v("collisionConf"));
  }
}

inline mc_rtc::Configuration scContactTaskToJSON(const StanceConfig::ContactTask & ct)
{
  mc_rtc::Configuration conf;
  conf.add("position", scPositionToJSON(ct.position));
  conf.add("orientation", scOrientationToJSON(ct.orientation));
  conf.add("linVel", scLinVelToJSON(ct.linVel));
  conf.add("waypointConf", scWaypointConfToJSON(ct.waypointConf));
  conf.add("collisionConf", scCollisionConfToJSON(ct.collisionConf));
  return conf;
}

inline void scContactObjFromJSON(StanceConfig::ContactObj & co, const mc_rtc::Configuration & v)
{
  v("posThresh", co.posThresh);
  v("velThresh", co.velThresh);
  v("adjustPosThresh", co.adjustPosThresh);
  v("adjustVelThresh", co.adjustVelThresh);
  v("adjustOriThresh", co.adjustOriThresh);
  v("adjustOffset", co.adjustOffset);
  v("adjustRPYOffset", co.adjustRPYOffset);
  v("adjustOriTBNWeight", co.adjustOriTBNWeight);
  v("preContactDist", co.preContactDist);
  v("gripperMoveAwayDist", co.gripperMoveAwayDist);
  v("useComplianceTask", co.useComplianceTask);
  v("complianceVelThresh", co.complianceVelThresh);
  v("complianceTargetTorque", co.complianceTargetTorque);
  v("complianceTargetForce", co.complianceTargetForce);
}

inline mc_rtc::Configuration scContactObjToJSON(const StanceConfig::ContactObj & co)
{
  mc_rtc::Configuration conf;
  conf.add("posThresh", co.posThresh);
  conf.add("velThresh", co.posThresh);
  conf.add("adjustPosThresh", co.adjustPosThresh);
  conf.add("adjustVelThresh", co.adjustVelThresh);
  conf.add("adjustOriThresh", co.adjustOriThresh);
  conf.add("adjustOffset", co.adjustOffset);
  conf.add("adjustOriTBNWeight", co.adjustOriTBNWeight);
  conf.add("preContactDist", co.preContactDist);
  return conf;
}

inline StanceConfig::BodiesCollisionConf scBodiesCollisionConfFromJSON(const mc_rtc::Configuration & v)
{
  StanceConfig::BodiesCollisionConf ret;
  v("body1", ret.body1);
  v("body2", ret.body2);
  if(v.has("collisionConf"))
  {
    scCollisionConfFromJSON(ret.collisionConf, v("collisionConf"));
  }
  return ret;
}

inline mc_rtc::Configuration scBodiesCollisionConfToJSON(const StanceConfig::BodiesCollisionConf & bcc)
{
  mc_rtc::Configuration conf;
  conf.add("body1", bcc.body1);
  conf.add("body2", bcc.body2);
  conf.add("collisionConf", scCollisionConfToJSON(bcc.collisionConf));
  return conf;
}

inline void scCollisionsFromJSON(StanceConfig::Collisions & cs, const mc_rtc::Configuration & v)
{
  if(v.has("autoc"))
  {
    auto autoc = v("autoc");
    for(size_t i = 0; i < autoc.size(); ++i)
    {
      auto c = autoc[i];
      cs.autoc.push_back(scBodiesCollisionConfFromJSON(c));
    }
  }
  if(v.has("robotEnv"))
  {
    auto robotEnv = v("robotEnv");
    for(size_t i = 0; i < robotEnv.size(); ++i)
    {
      auto c = robotEnv[i];
      cs.robotEnv.push_back(scBodiesCollisionConfFromJSON(c));
    }
  }
  if(v.has("robotEnvContactFilter"))
  {
    auto rEnvCF = v("robotEnvContactFilter");
    for(size_t i = 0; i < rEnvCF.size(); ++i)
    {
      auto f = rEnvCF[i];
      std::pair<std::string, std::string> key(f("surf1"),f("surf2"));
      if(cs.robotEnvContactFilter.count(key) == 0)
      {
        cs.robotEnvContactFilter[key] = {};
      }
      auto filtereds = f("filtered");
      for(size_t i = 0; i < filtereds.size(); ++i)
      {
        auto filtered = filtereds[i];
        cs.robotEnvContactFilter[key].push_back(std::pair<std::string, std::string>(filtered("body1"), filtered("body2")));
      }
    }
  }
}

inline mc_rtc::Configuration scCollisionsToJSON(const StanceConfig::Collisions & cs)
{
  mc_rtc::Configuration conf;
  mc_rtc::Configuration autoc = conf.array("autoc");
  for(const auto & bc : cs.autoc)
  {
    autoc.push(scBodiesCollisionConfToJSON(bc));
  }
  mc_rtc::Configuration robotEnv = conf.array("robotEnv");
  for(const auto & bc : cs.robotEnv)
  {
    robotEnv.push(scBodiesCollisionConfToJSON(bc));
  }
  mc_rtc::Configuration robotEnvContactFilter = conf.array("robotEnvContactFilter");
  for(const auto & recf : cs.robotEnvContactFilter)
  {
    mc_rtc::Configuration vv;
    vv.add("surf1", recf.first.first);
    vv.add("surf2", recf.first.second);
    mc_rtc::Configuration filtered = vv.array("filtered");
    for(const auto & p : recf.second)
    {
      mc_rtc::Configuration vvv;
      vvv.add("body1", p.first);
      vvv.add("body2", p.second);
      filtered.push(vvv);
    }
    robotEnvContactFilter.push(vv);
  }
  return conf;
}

inline void StanceConfigFromJSON(StanceConfig & sc, const mc_rtc::Configuration & v)
{
  if(v.has("comTask"))
  {
    scCoMTaskFromJSON(sc.comTask, v("comTask"));
  }
  if(v.has("comObj"))
  {
    scCoMObjFromJSON(sc.comObj, v("comObj"));
  }
  if(v.has("postureTask"))
  {
    scPostureTaskFromJSON(sc.postureTask, v("postureTask"));
  }
  if(v.has("contactTask"))
  {
    scContactTaskFromJSON(sc.contactTask, v("contactTask"));
  }
  if(v.has("contactObj"))
  {
    scContactObjFromJSON(sc.contactObj, v("contactObj"));
  }
  if(v.has("collisions"))
  {
    scCollisionsFromJSON(sc.collisions, v("collisions"));
  }
}

inline mc_rtc::Configuration StanceConfigToJSON(const StanceConfig & sc)
{
  mc_rtc::Configuration conf;
  conf.add("comTask", scCoMTaskToJSON(sc.comTask));
  conf.add("comObj", scCoMObjToJSON(sc.comObj));
  conf.add("postureTask", scPostureTaskToJSON(sc.postureTask));
  conf.add("contactTask", scContactTaskToJSON(sc.contactTask));
  conf.add("contactObj", scContactObjToJSON(sc.contactObj));
  conf.add("collisions", scCollisionsToJSON(sc.collisions));
  return conf;
}

}
