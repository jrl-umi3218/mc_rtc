#pragma once

/* A collection of functions for JSON manipulation of
 * StanceConfig objects */

#include <mc_rbdyn/StanceConfig.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include "../../mc_rtc/internals/json.h"

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

inline rapidjson::Value scCoMTaskToJSON(const StanceConfig::CoMTask& ct, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("stiffness", ct.stiffness, allocator);
  ret.AddMember("extraStiffness", ct.extraStiffness, allocator);
  ret.AddMember("weight", ct.weight, allocator);
  ret.AddMember("targetSpeed", ct.targetSpeed, allocator);
  return ret;
}

inline void scCoMObjFromJSON(StanceConfig::CoMObj & ret, const mc_rtc::Configuration & v)
{
  v("posThresh", ret.posThresh);
  v("velThresh", ret.velThresh);
  v("comOffset", ret.comOffset);
  v("comAdjustOffset", ret.comAdjustOffset);
  v("timeout", ret.timeout);
}

inline rapidjson::Value scCoMObjToJSON(const StanceConfig::CoMObj & co, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("posThresh", co.posThresh, allocator);
  ret.AddMember("velThresh", co.velThresh, allocator);
  rapidjson::Value cOff(rapidjson::kArrayType);
  for(int i = 0; i < 3; ++i)
  {
    cOff.PushBack(co.comOffset(i), allocator);
  }
  ret.AddMember("comOffset", cOff, allocator);
  return ret;
}

inline void scPostureTaskFromJSON(StanceConfig::PostureTask & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("weight", ret.weight);
}

inline rapidjson::Value scPostureTaskToJSON(const StanceConfig::PostureTask & pt, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  v.AddMember("stiffness", pt.stiffness, allocator);
  v.AddMember("weight", pt.weight, allocator);
  return v;
}

inline void scPositionFromJSON(StanceConfig::Position & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("extraStiffness", ret.extraStiffness);
  v("weight", ret.weight);
  v("targetSpeed", ret.targetSpeed);
}

inline rapidjson::Value scPositionToJSON(const StanceConfig::Position& ct, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("stiffness", ct.stiffness, allocator);
  ret.AddMember("extraStiffness", ct.extraStiffness, allocator);
  ret.AddMember("weight", ct.weight, allocator);
  ret.AddMember("targetSpeed", ct.targetSpeed, allocator);
  return ret;
}

inline void scOrientationFromJSON(StanceConfig::Orientation & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("finalWeight", ret.finalWeight);
  v("weight", ret.weight);
}

inline rapidjson::Value scOrientationToJSON(const StanceConfig::Orientation& ct, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("stiffness", ct.stiffness, allocator);
  ret.AddMember("weight", ct.weight, allocator);
  ret.AddMember("finalWeight", ct.finalWeight, allocator);
  return ret;
}

inline void scLinVelFromJSON(StanceConfig::LinVel & ret, const mc_rtc::Configuration & v)
{
  v("stiffness", ret.stiffness);
  v("weight", ret.weight);
  v("speed", ret.speed);
}

inline rapidjson::Value scLinVelToJSON(const StanceConfig::LinVel& ct, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("stiffness", ct.stiffness, allocator);
  ret.AddMember("weight", ct.weight, allocator);
  ret.AddMember("speed", ct.speed, allocator);
  return ret;
}

inline void scWaypointConfFromJSON(StanceConfig::WaypointConf & ret, const mc_rtc::Configuration & v)
{
  v("skip", ret.skip);
  v("thresh", ret.thresh);
  if(v.has("type"))
  {
    if(v("type") == "percentWaypoint")
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
    else if(v("type") == "hardCodedPos")
    {
      double x = v("conf")("x");
      double y = v("conf")("y");
      double z = v("conf")("z");
      ret.pos = hardCodedPos(x, y, z);
    }
    else
    {
      LOG_ERROR("Invalid waypoint type: " << v("type"))
      throw(std::string("Invalid waypoint type in JSON string"));
    }
  }
}

/*FIXME Not very clean and should not be used in practice */
inline rapidjson::Value scWaypointConfToJSON(const StanceConfig::WaypointConf & wpc, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("thresh", wpc.thresh, allocator);
  ret.AddMember("conf", rapidjson::Value(rapidjson::kObjectType), allocator);
  /* First check if it is an hardcoded pos */
  sva::PTransformd start = sva::PTransformd::Identity();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  sva::PTransformd pt1 = sva::PTransformd(Eigen::Vector3d(1,1,1));
  sva::PTransformd pt2 = sva::PTransformd(Eigen::Vector3d(2,2,2));
  Eigen::Vector3d pos1 = wpc.pos(start, pt1, N);
  Eigen::Vector3d pos2 = wpc.pos(start, pt2, N);
  if(pos1 == pos2)
  {
    ret.AddMember("type", "hardCodedPos", allocator);
    rapidjson::Value conf(rapidjson::kObjectType);
    conf.AddMember("x", pos1.x(), allocator);
    conf.AddMember("y", pos1.y(), allocator);
    conf.AddMember("z", pos1.z(), allocator);
    ret.AddMember("conf", conf, allocator);
  }
  else
  {
    ret.AddMember("type", "percentWaypoint", allocator);
    rapidjson::Value conf(rapidjson::kObjectType);
    conf.AddMember("x", pos1.x(), allocator);
    conf.AddMember("y", pos1.y(), allocator);
    conf.AddMember("z", pos1.z(), allocator);
    N(0) = 1;
    pos1 = wpc.pos(start, start, N);
    conf.AddMember("nOff", pos1.x(), allocator);
    ret.AddMember("conf", conf, allocator);
  }
  return ret;
}

inline void scCollisionConfFromJSON(StanceConfig::CollisionConf & cc, const mc_rtc::Configuration & v)
{
  v("iDist", cc.iDist);
  v("sDist", cc.sDist);
  v("damping", cc.damping);
}

inline rapidjson::Value scCollisionConfToJSON(const StanceConfig::CollisionConf & cc, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  v.AddMember("iDist", cc.iDist, allocator);
  v.AddMember("sDist", cc.sDist, allocator);
  v.AddMember("damping", cc.damping, allocator);
  return v;
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

inline rapidjson::Value scContactTaskToJSON(const StanceConfig::ContactTask & ct, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  v.AddMember("position", scPositionToJSON(ct.position, allocator).Move(), allocator);
  v.AddMember("orientation", scOrientationToJSON(ct.orientation, allocator).Move(), allocator);
  v.AddMember("linVel", scLinVelToJSON(ct.linVel, allocator).Move(), allocator);
  v.AddMember("waypointConf", scWaypointConfToJSON(ct.waypointConf, allocator).Move(), allocator);
  v.AddMember("collisionConf", scCollisionConfToJSON(ct.collisionConf, allocator).Move(), allocator);
  return v;
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

inline rapidjson::Value scContactObjToJSON(const StanceConfig::ContactObj & co, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  v.AddMember("posThresh", co.posThresh, allocator);
  v.AddMember("velThresh", co.posThresh, allocator);
  v.AddMember("adjustPosThresh", co.adjustPosThresh, allocator);
  v.AddMember("adjustVelThresh", co.adjustVelThresh, allocator);
  v.AddMember("adjustOriThresh", co.adjustOriThresh, allocator);
  rapidjson::Value adjustOffset(rapidjson::kArrayType);
  adjustOffset.PushBack(co.adjustOffset.x(), allocator);
  adjustOffset.PushBack(co.adjustOffset.y(), allocator);
  adjustOffset.PushBack(co.adjustOffset.z(), allocator);
  v.AddMember("adjustOffset", adjustOffset, allocator);
  rapidjson::Value adjustOriTBNWeight(rapidjson::kArrayType);
  adjustOriTBNWeight.PushBack(co.adjustOriTBNWeight.x(), allocator);
  adjustOriTBNWeight.PushBack(co.adjustOriTBNWeight.y(), allocator);
  adjustOriTBNWeight.PushBack(co.adjustOriTBNWeight.z(), allocator);
  v.AddMember("adjustOriTBNWeight", adjustOriTBNWeight, allocator);
  v.AddMember("preContactDist", co.preContactDist, allocator);
  return v;
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

inline rapidjson::Value scBodiesCollisionConfToJSON(const StanceConfig::BodiesCollisionConf & bcc, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  v.AddMember("body1", bcc.body1, allocator);
  v.AddMember("body2", bcc.body2, allocator);
  v.AddMember("collisionConf", scCollisionConfToJSON(bcc.collisionConf, allocator).Move(), allocator);
  return v;
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

inline rapidjson::Value scCollisionsToJSON(const StanceConfig::Collisions & cs, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  rapidjson::Value autoc(rapidjson::kArrayType);
  for(const auto & bc : cs.autoc)
  {
    autoc.PushBack(scBodiesCollisionConfToJSON(bc, allocator), allocator);
  }
  v.AddMember("autoc", autoc, allocator);
  rapidjson::Value robotEnv(rapidjson::kArrayType);
  for(const auto & bc : cs.robotEnv)
  {
    robotEnv.PushBack(scBodiesCollisionConfToJSON(bc, allocator), allocator);
  }
  v.AddMember("robotEnv", robotEnv, allocator);
  rapidjson::Value robotEnvContactFilter(rapidjson::kArrayType);
  for(const auto & recf : cs.robotEnvContactFilter)
  {
    rapidjson::Value vv(rapidjson::kObjectType);
    vv.AddMember("surf1", recf.first.first, allocator);
    vv.AddMember("surf2", recf.first.second, allocator);
    vv.AddMember("filtered", rapidjson::Value(rapidjson::kArrayType).Move(), allocator);
    for(const auto & p : recf.second)
    {
      rapidjson::Value vvv(rapidjson::kObjectType);
      vvv.AddMember("body1", p.first, allocator);
      vvv.AddMember("body2", p.second, allocator);
      vv["filtered"].PushBack(vvv, allocator);
    }
    robotEnvContactFilter.PushBack(vv, allocator);
  }
  v.AddMember("robotEnvContactFilter", robotEnvContactFilter, allocator);
  return v;
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

inline rapidjson::Value StanceConfigToJSON(const StanceConfig & sc, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  v.AddMember("comTask", scCoMTaskToJSON(sc.comTask, allocator).Move(), allocator);
  v.AddMember("comObj", scCoMObjToJSON(sc.comObj, allocator).Move(), allocator);
  v.AddMember("postureTask", scPostureTaskToJSON(sc.postureTask, allocator).Move(), allocator);
  v.AddMember("contactTask", scContactTaskToJSON(sc.contactTask, allocator).Move(), allocator);
  v.AddMember("contactObj", scContactObjToJSON(sc.contactObj, allocator).Move(), allocator);
  v.AddMember("collisions", scCollisionsToJSON(sc.collisions, allocator).Move(), allocator);
  return v;
}

}
