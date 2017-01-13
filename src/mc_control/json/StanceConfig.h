#pragma once

/* A collection of functions for JSON manipulation of
 * StanceConfig objects */

#include <mc_rbdyn/StanceConfig.h>

#include <mc_rtc/logging.h>

#define RAPIDJSON_HAS_STDSTRING 1
#define RAPIDJSON_PARSE_DEFAULT_FLAGS rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag
#include "rapidjson/document.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/writer.h"
#include "rapidjson/error/en.h"

#include <iostream>

namespace mc_rbdyn
{

inline void scCoMTaskFromJSON(StanceConfig::CoMTask & ret, const rapidjson::Value & v)
{
  if(v.HasMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].GetDouble();
  }
  if(v.HasMember("extraStiffness"))
  {
    ret.extraStiffness = v["extraStiffness"].GetDouble();
  }
  if(v.HasMember("weight"))
  {
    ret.weight = v["weight"].GetDouble();
  }
  if(v.HasMember("targetSpeed"))
  {
    ret.targetSpeed = v["targetSpeed"].GetDouble();
  }
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

inline void scCoMObjFromJSON(StanceConfig::CoMObj & ret, const rapidjson::Value &v)
{
  if(v.HasMember("posThresh"))
  {
    ret.posThresh = v["posThresh"].GetDouble();
  }
  if(v.HasMember("velThresh"))
  {
    ret.velThresh = v["velThresh"].GetDouble();
  }
  if(v.HasMember("comOffset"))
  {
    for(int i = 0; i < 3; ++i)
    {
      ret.comOffset(i) = v["comOffset"][i].GetDouble();
    }
  }
  if(v.HasMember("comAdjustOffset"))
  {
    for(int i = 0; i < 3; ++i)
    {
      ret.comAdjustOffset(i) = v["comAdjustOffset"][i].GetDouble();
    }
  }
  if(v.HasMember("timeout"))
  {
    ret.timeout = v["timeout"].GetDouble();
  }
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

inline void scPostureTaskFromJSON(StanceConfig::PostureTask & ret, const rapidjson::Value & v)
{
  if(v.HasMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].GetDouble();
  }
  if(v.HasMember("weight"))
  {
    ret.weight = v["weight"].GetDouble();
  }
}

inline rapidjson::Value scPostureTaskToJSON(const StanceConfig::PostureTask & pt, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  v.AddMember("stiffness", pt.stiffness, allocator);
  v.AddMember("weight", pt.weight, allocator);
  return v;
}

inline void scPositionFromJSON(StanceConfig::Position & ret, const rapidjson::Value & v)
{
  if(v.HasMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].GetDouble();
  }
  if(v.HasMember("extraStiffness"))
  {
    ret.extraStiffness = v["extraStiffness"].GetDouble();
  }
  if(v.HasMember("weight"))
  {
    ret.weight = v["weight"].GetDouble();
  }
  if(v.HasMember("targetSpeed"))
  {
    ret.targetSpeed = v["targetSpeed"].GetDouble();
  }
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

inline void scOrientationFromJSON(StanceConfig::Orientation & ret, const rapidjson::Value & v)
{
  if(v.HasMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].GetDouble();
  }
  if(v.HasMember("finalWeight"))
  {
    ret.finalWeight = v["finalWeight"].GetDouble();
  }
  if(v.HasMember("weight"))
  {
    ret.weight = v["weight"].GetDouble();
  }
}

inline rapidjson::Value scOrientationToJSON(const StanceConfig::Orientation& ct, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("stiffness", ct.stiffness, allocator);
  ret.AddMember("weight", ct.weight, allocator);
  ret.AddMember("finalWeight", ct.finalWeight, allocator);
  return ret;
}

inline void scLinVelFromJSON(StanceConfig::LinVel & ret, const rapidjson::Value & v)
{
  if(v.HasMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].GetDouble();
  }
  if(v.HasMember("weight"))
  {
    ret.weight = v["weight"].GetDouble();
  }
  if(v.HasMember("speed"))
  {
    ret.speed = v["speed"].GetDouble();
  }
}

inline rapidjson::Value scLinVelToJSON(const StanceConfig::LinVel& ct, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("stiffness", ct.stiffness, allocator);
  ret.AddMember("weight", ct.weight, allocator);
  ret.AddMember("speed", ct.speed, allocator);
  return ret;
}

inline void scWaypointConfFromJSON(StanceConfig::WaypointConf & ret, const rapidjson::Value & v)
{
  if(v.HasMember("skip"))
  {
    ret.skip = v["skip"].GetBool();
  }
  if(v.HasMember("thresh"))
  {
    ret.thresh = v["thresh"].GetDouble();
  }
  if(v.HasMember("type"))
  {
    if(v["type"] == "percentWaypoint")
    {
      double x = v["conf"]["x"].GetDouble();
      double y = v["conf"]["y"].GetDouble();
      double z = v["conf"]["z"].GetDouble();
      double nOff = v["conf"]["nOff"].GetDouble();
      double xOff = 0;
      if(v["conf"].HasMember("xOff"))
      {
        xOff = v["conf"]["xOff"].GetDouble();
      }
      double yOff = 0;
      if(v["conf"].HasMember("yOff"))
      {
        yOff = v["conf"]["yOff"].GetDouble();
      }
      double zOff = 0;
      if(v["conf"].HasMember("zOff"))
      {
        zOff = v["conf"]["zOff"].GetDouble();
      }
      ret.pos = percentWaypoint(x, y, z, nOff, xOff, yOff, zOff);
    }
    else if(v["type"] == "hardCodedPos")
    {
      double x = v["conf"]["x"].GetDouble();
      double y = v["conf"]["y"].GetDouble();
      double z = v["conf"]["z"].GetDouble();
      ret.pos = hardCodedPos(x, y, z);
    }
    else
    {
      LOG_ERROR("Invalid waypoint type: " << v["type"].GetString())
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

inline void scCollisionConfFromJSON(StanceConfig::CollisionConf & cc, const rapidjson::Value & v)
{
  if(v.HasMember("iDist"))
  {
    cc.iDist = v["iDist"].GetDouble();
  }
  if(v.HasMember("sDist"))
  {
    cc.sDist = v["sDist"].GetDouble();
  }
  if(v.HasMember("damping"))
  {
    cc.damping = v["damping"].GetDouble();
  }
}

inline rapidjson::Value scCollisionConfToJSON(const StanceConfig::CollisionConf & cc, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value v(rapidjson::kObjectType);
  v.AddMember("iDist", cc.iDist, allocator);
  v.AddMember("sDist", cc.sDist, allocator);
  v.AddMember("damping", cc.damping, allocator);
  return v;
}

inline void scContactTaskFromJSON(StanceConfig::ContactTask & ct, const rapidjson::Value & v)
{
  if(v.HasMember("position"))
  {
    scPositionFromJSON(ct.position, v["position"]);
  }
  if(v.HasMember("orientation"))
  {
     scOrientationFromJSON(ct.orientation, v["orientation"]);
  }
  if(v.HasMember("linVel"))
  {
    scLinVelFromJSON(ct.linVel, v["linVel"]);
  }
  if(v.HasMember("waypointConf"))
  {
    scWaypointConfFromJSON(ct.waypointConf, v["waypointConf"]);
  }
  if(v.HasMember("collisionConf"))
  {
    scCollisionConfFromJSON(ct.collisionConf, v["collisionConf"]);
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

inline void scContactObjFromJSON(StanceConfig::ContactObj & co, const rapidjson::Value & v)
{
  if(v.HasMember("posThresh"))
  {
    co.posThresh = v["posThresh"].GetDouble();
  }
  if(v.HasMember("velThresh"))
  {
    co.velThresh = v["velThresh"].GetDouble();
  }
  if(v.HasMember("adjustPosThresh"))
  {
    co.adjustPosThresh = v["adjustPosThresh"].GetDouble();
  }
  if(v.HasMember("adjustVelThresh"))
  {
    co.adjustVelThresh = v["adjustVelThresh"].GetDouble();
  }
  if(v.HasMember("adjustOriThresh"))
  {
    co.adjustOriThresh = v["adjustOriThresh"].GetDouble();
  }
  if(v.HasMember("adjustOffset"))
  {
    co.adjustOffset.x() = v["adjustOffset"][0].GetDouble();
    co.adjustOffset.y() = v["adjustOffset"][1].GetDouble();
    co.adjustOffset.z() = v["adjustOffset"][2].GetDouble();
  }
  if(v.HasMember("adjustRPYOffset"))
  {
    co.adjustRPYOffset.x() = v["adjustRPYOffset"][0].GetDouble();
    co.adjustRPYOffset.y() = v["adjustRPYOffset"][1].GetDouble();
    co.adjustRPYOffset.z() = v["adjustRPYOffset"][2].GetDouble();
  }
  if(v.HasMember("adjustOriTBNWeight"))
  {
    co.adjustOriTBNWeight.x() = v["adjustOriTBNWeight"][0].GetDouble();
    co.adjustOriTBNWeight.y() = v["adjustOriTBNWeight"][1].GetDouble();
    co.adjustOriTBNWeight.z() = v["adjustOriTBNWeight"][2].GetDouble();
  }
  if(v.HasMember("preContactDist"))
  {
    co.preContactDist = v["preContactDist"].GetDouble();
  }
  if(v.HasMember("gripperMoveAwayDist"))
  {
    co.gripperMoveAwayDist = v["gripperMoveAwayDist"].GetDouble();
  }
  if(v.HasMember("useComplianceTask"))
  {
    co.useComplianceTask = v["useComplianceTask"].GetBool();
  }
  if(v.HasMember("complianceVelThresh"))
  {
    co.complianceVelThresh = v["complianceVelThresh"].GetDouble();
  }
  if(v.HasMember("complianceTargetTorque"))
  {
    co.complianceTargetTorque.x() = v["complianceTargetTorque"][0].GetDouble();
    co.complianceTargetTorque.y() = v["complianceTargetTorque"][1].GetDouble();
    co.complianceTargetTorque.z() = v["complianceTargetTorque"][2].GetDouble();
  }
  if(v.HasMember("complianceTargetForce"))
  {
    co.complianceTargetForce.x() = v["complianceTargetForce"][0].GetDouble();
    co.complianceTargetForce.y() = v["complianceTargetForce"][1].GetDouble();
    co.complianceTargetForce.z() = v["complianceTargetForce"][2].GetDouble();
  }
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

inline StanceConfig::BodiesCollisionConf scBodiesCollisionConfFromJSON(const rapidjson::Value & v)
{
  StanceConfig::BodiesCollisionConf ret;
  ret.body1 = v["body1"].GetString();
  ret.body2 = v["body2"].GetString();
  if(v.HasMember("collisionConf"))
  {
    scCollisionConfFromJSON(ret.collisionConf, v["collisionConf"]);
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

inline void scCollisionsFromJSON(StanceConfig::Collisions & cs, const rapidjson::Value & v)
{
  if(v.HasMember("autoc"))
  {
    for(const auto & c : v["autoc"].GetArray())
    {
      cs.autoc.push_back(scBodiesCollisionConfFromJSON(c));
    }
  }
  if(v.HasMember("robotEnv"))
  {
    for(const auto & c : v["robotEnv"].GetArray())
    {
      cs.robotEnv.push_back(scBodiesCollisionConfFromJSON(c));
    }
  }
  if(v.HasMember("robotEnvContactFilter"))
  {
    for(const auto & f : v["robotEnvContactFilter"].GetArray())
    {
      std::pair<std::string, std::string> key(f["surf1"].GetString(),f["surf2"].GetString());
      if(cs.robotEnvContactFilter.count(key) == 0)
      {
        cs.robotEnvContactFilter[key] = {};
      }
      for(const auto & filtered : f["filtered"].GetArray())
      {
        cs.robotEnvContactFilter[key].push_back(std::pair<std::string, std::string>(filtered["body1"].GetString(), filtered["body2"].GetString()));
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

inline void StanceConfigFromJSON(StanceConfig & sc, const rapidjson::Value & v)
{
  if(v.HasMember("comTask"))
  {
    scCoMTaskFromJSON(sc.comTask, v["comTask"]);
  }
  if(v.HasMember("comObj"))
  {
    scCoMObjFromJSON(sc.comObj, v["comObj"]);
  }
  if(v.HasMember("postureTask"))
  {
    scPostureTaskFromJSON(sc.postureTask, v["postureTask"]);
  }
  if(v.HasMember("contactTask"))
  {
    scContactTaskFromJSON(sc.contactTask, v["contactTask"]);
  }
  if(v.HasMember("contactObj"))
  {
    scContactObjFromJSON(sc.contactObj, v["contactObj"]);
  }
  if(v.HasMember("collisions"))
  {
    scCollisionsFromJSON(sc.collisions, v["collisions"]);
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
