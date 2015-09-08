#pragma once

/* A collection of functions for JSON manipulation of
 * StanceConfig objects */

#include <mc_rbdyn/StanceConfig.h>

#include <json/json.h>
#include <iostream>

namespace mc_rbdyn
{

inline void scCoMTaskFromJSON(StanceConfig::CoMTask & ret, const Json::Value & v)
{
  if(v.isMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].asDouble();
  }
  if(v.isMember("extraStiffness"))
  {
    ret.extraStiffness = v["extraStiffness"].asDouble();
  }
  if(v.isMember("weight"))
  {
    ret.weight = v["weight"].asDouble();
  }
  if(v.isMember("targetSpeed"))
  {
    ret.targetSpeed = v["targetSpeed"].asDouble();
  }
}

inline Json::Value scCoMTaskToJSON(const StanceConfig::CoMTask& ct)
{
  Json::Value ret(Json::objectValue);
  ret["stiffness"] = ct.stiffness;
  ret["extraStiffness"] = ct.extraStiffness;
  ret["weight"] = ct.weight;
  ret["targetSpeed"] = ct.targetSpeed;
  return ret;
}

inline void scCoMObjFromJSON(StanceConfig::CoMObj & ret, const Json::Value &v)
{
  if(v.isMember("posThresh"))
  {
    ret.posThresh = v["posThresh"].asDouble();
  }
  if(v.isMember("velThresh"))
  {
    ret.velThresh = v["velThresh"].asDouble();
  }
  if(v.isMember("comOffset"))
  {
    for(Json::Value::ArrayIndex i = 0; i < 3; ++i)
    {
      ret.comOffset(i) = v["comOffset"][i].asDouble();
    }
  }
}

inline Json::Value scCoMObjToJSON(const StanceConfig::CoMObj & co)
{
  Json::Value ret(Json::objectValue);
  ret["posThresh"] = co.posThresh;
  ret["velThresh"] = co.velThresh;
  Json::Value cOff(Json::arrayValue);
  for(size_t i = 0; i < 3; ++i)
  {
    cOff.append(Json::Value(co.comOffset(i)));
  }
  ret["comOffset"] = cOff;
  return ret;
}

inline void scPostureTaskFromJSON(StanceConfig::PostureTask & ret, const Json::Value & v)
{
  if(v.isMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].asDouble();
  }
  if(v.isMember("weight"))
  {
    ret.weight = v["weight"].asDouble();
  }
}

inline Json::Value scPostureTaskToJSON(const StanceConfig::PostureTask & pt)
{
  Json::Value v(Json::objectValue);
  v["stiffness"] = pt.stiffness;
  v["weight"] = pt.weight;
  return v;
}

inline void scPositionFromJSON(StanceConfig::Position & ret, const Json::Value & v)
{
  if(v.isMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].asDouble();
  }
  if(v.isMember("extraStiffness"))
  {
    ret.extraStiffness = v["extraStiffness"].asDouble();
  }
  if(v.isMember("weight"))
  {
    ret.weight = v["weight"].asDouble();
  }
  if(v.isMember("targetSpeed"))
  {
    ret.targetSpeed = v["targetSpeed"].asDouble();
  }
}

inline Json::Value scPositionToJSON(const StanceConfig::Position& ct)
{
  Json::Value ret(Json::objectValue);
  ret["stiffness"] = ct.stiffness;
  ret["extraStiffness"] = ct.extraStiffness;
  ret["weight"] = ct.weight;
  ret["targetSpeed"] = ct.targetSpeed;
  return ret;
}

inline void scOrientationFromJSON(StanceConfig::Orientation & ret, const Json::Value & v)
{
  if(v.isMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].asDouble();
  }
  if(v.isMember("finalWeight"))
  {
    ret.finalWeight = v["finalWeight"].asDouble();
  }
  if(v.isMember("weight"))
  {
    ret.weight = v["weight"].asDouble();
  }
}

inline Json::Value scOrientationToJSON(const StanceConfig::Orientation& ct)
{
  Json::Value ret(Json::objectValue);
  ret["stiffness"] = ct.stiffness;
  ret["weight"] = ct.weight;
  ret["finalWeight"] = ct.finalWeight;
  return ret;
}

inline void scLinVelFromJSON(StanceConfig::LinVel & ret, const Json::Value & v)
{
  if(v.isMember("stiffness"))
  {
    ret.stiffness = v["stiffness"].asDouble();
  }
  if(v.isMember("weight"))
  {
    ret.weight = v["weight"].asDouble();
  }
  if(v.isMember("speed"))
  {
    ret.speed = v["speed"].asDouble();
  }
}

inline Json::Value scLinVelToJSON(const StanceConfig::LinVel& ct)
{
  Json::Value ret(Json::objectValue);
  ret["stiffness"] = ct.stiffness;
  ret["weight"] = ct.weight;
  ret["speed"] = ct.speed;
  return ret;
}

inline void scWaypointConfFromJSON(StanceConfig::WaypointConf & ret, const Json::Value & v)
{
  if(v.isMember("thresh"))
  {
    ret.thresh = v["thresh"].asDouble();
  }
  if(v.isMember("type"))
  {
    if(v["type"] == "percentWaypoint")
    {
      double x = v["conf"]["x"].asDouble();
      double y = v["conf"]["y"].asDouble();
      double z = v["conf"]["z"].asDouble();
      double nOff = v["conf"]["nOff"].asDouble();
      ret.pos = percentWaypoint(x, y, z, nOff);
    }
    else if(v["type"] == "hardCodedPos")
    {
      double x = v["conf"]["x"].asDouble();
      double y = v["conf"]["y"].asDouble();
      double z = v["conf"]["z"].asDouble();
      ret.pos = hardCodedPos(x, y, z);
    }
    else
    {
      std::cerr << "Invalid waypoint type: " << v["type"] << std::endl;
      throw(std::string("Invalid waypoint type in JSON string"));
    }
  }
}

/*FIXME Not very clean and should not be used in practice */
inline Json::Value scWaypointConfToJSON(const StanceConfig::WaypointConf & wpc)
{
  Json::Value ret(Json::objectValue);
  ret["thresh"] = wpc.thresh;
  ret["conf"] = Json::Value(Json::objectValue);
  /* First check if it is an hardcoded pos */
  sva::PTransformd start = sva::PTransformd::Identity();
  Eigen::Vector3d N = Eigen::Vector3d::Zero();
  sva::PTransformd pt1 = sva::PTransformd(Eigen::Vector3d(1,1,1));
  sva::PTransformd pt2 = sva::PTransformd(Eigen::Vector3d(2,2,2));
  Eigen::Vector3d pos1 = wpc.pos(start, pt1, N);
  Eigen::Vector3d pos2 = wpc.pos(start, pt2, N);
  if(pos1 == pos2)
  {
    ret["type"] = "hardCodedPos";
    Json::Value conf(Json::objectValue);
    conf["x"] = pos1.x();
    conf["y"] = pos1.y();
    conf["z"] = pos1.z();
    ret["conf"] = conf;
  }
  else
  {
    ret["type"] = "percentWaypoint";
    Json::Value conf(Json::objectValue);
    conf["x"] = pos1.x();
    conf["y"] = pos1.y();
    conf["z"] = pos1.z();
    N(0) = 1;
    pos1 = wpc.pos(start, start, N);
    conf["nOff"] = pos1.x();
    ret["conf"] = conf;
  }
  return ret;
}

inline void scCollisionConfFromJSON(StanceConfig::CollisionConf & cc, const Json::Value & v)
{
  if(v.isMember("iDist"))
  {
    cc.iDist = v["iDist"].asDouble();
  }
  if(v.isMember("sDist"))
  {
    cc.sDist = v["sDist"].asDouble();
  }
  if(v.isMember("damping"))
  {
    cc.damping = v["damping"].asDouble();
  }
}

inline Json::Value scCollisionConfToJSON(const StanceConfig::CollisionConf & cc)
{
  Json::Value v(Json::objectValue);
  v["iDist"] = cc.iDist;
  v["sDist"] = cc.sDist;
  v["damping"] = cc.damping;
  return v;
}

inline void scContactTaskFromJSON(StanceConfig::ContactTask & ct, const Json::Value & v)
{
  if(v.isMember("position"))
  {
    scPositionFromJSON(ct.position, v["position"]);
  }
  if(v.isMember("orientation"))
  {
     scOrientationFromJSON(ct.orientation, v["orientation"]);
  }
  if(v.isMember("linVel"))
  {
    scLinVelFromJSON(ct.linVel, v["linVel"]);
  }
  if(v.isMember("waypointConf"))
  {
    scWaypointConfFromJSON(ct.waypointConf, v["waypointConf"]);
  }
  if(v.isMember("collisionConf"))
  {
    scCollisionConfFromJSON(ct.collisionConf, v["collisionConf"]);
  }
}

inline Json::Value scContactTaskToJSON(const StanceConfig::ContactTask & ct)
{
  Json::Value v(Json::objectValue);
  v["position"] = scPositionToJSON(ct.position);
  v["orientation"] = scOrientationToJSON(ct.orientation);
  v["linVel"] = scLinVelToJSON(ct.linVel);
  v["waypointConf"] = scWaypointConfToJSON(ct.waypointConf);
  v["collisionConf"] = scCollisionConfToJSON(ct.collisionConf);
  return v;
}

inline void scContactObjFromJSON(StanceConfig::ContactObj & co, const Json::Value & v)
{
  if(v.isMember("posThresh"))
  {
    co.posThresh = v["posThresh"].asDouble();
  }
  if(v.isMember("velThresh"))
  {
    co.velThresh = v["velThresh"].asDouble();
  }
  if(v.isMember("adjustPosThresh"))
  {
    co.adjustPosThresh = v["adjustPosThresh"].asDouble();
  }
  if(v.isMember("adjustVelThresh"))
  {
    co.adjustVelThresh = v["adjustVelThresh"].asDouble();
  }
  if(v.isMember("adjustOriThresh"))
  {
    co.adjustOriThresh = v["adjustOriThresh"].asDouble();
  }
  if(v.isMember("adjustOffset"))
  {
    co.adjustOffset.x() = v["adjustOffset"][0].asDouble();
    co.adjustOffset.y() = v["adjustOffset"][1].asDouble();
    co.adjustOffset.z() = v["adjustOffset"][2].asDouble();
  }
  if(v.isMember("adjustOriTBNWeight"))
  {
    co.adjustOriTBNWeight.x() = v["adjustOriTBNWeight"][0].asDouble();
    co.adjustOriTBNWeight.y() = v["adjustOriTBNWeight"][1].asDouble();
    co.adjustOriTBNWeight.z() = v["adjustOriTBNWeight"][2].asDouble();
  }
  if(v.isMember("preContactDist"))
  {
    co.preContactDist = v["preContactDist"].asDouble();
  }
}

inline Json::Value scContactObjToJSON(const StanceConfig::ContactObj & co)
{
  Json::Value v(Json::objectValue);
  v["posThresh"] = co.posThresh;
  v["velThresh"] = co.posThresh;
  v["adjustPosThresh"] = co.adjustPosThresh;
  v["adjustVelThresh"] = co.adjustVelThresh;
  v["adjustOriThresh"] = co.adjustOriThresh;
  v["adjustOffset"] = Json::Value(Json::arrayValue);
  v["adjustOffset"].append(co.adjustOffset.x());
  v["adjustOffset"].append(co.adjustOffset.y());
  v["adjustOffset"].append(co.adjustOffset.z());
  v["adjustOriTBNWeight"] = Json::Value(Json::arrayValue);
  v["adjustOriTBNWeight"].append(co.adjustOriTBNWeight.x());
  v["adjustOriTBNWeight"].append(co.adjustOriTBNWeight.y());
  v["adjustOriTBNWeight"].append(co.adjustOriTBNWeight.z());
  v["preContactDist"] = co.preContactDist;
  return v;
}

inline StanceConfig::BodiesCollisionConf scBodiesCollisionConfFromJSON(const Json::Value & v)
{
  StanceConfig::BodiesCollisionConf ret;
  ret.body1 = v["body1"].asString();
  ret.body2 = v["body2"].asString();
  if(v.isMember("collisionConf"))
  {
    scCollisionConfFromJSON(ret.collisionConf, v["collisionConf"]);
  }
  return ret;
}

inline Json::Value scBodiesCollisionConfToJSON(const StanceConfig::BodiesCollisionConf & bcc)
{
  Json::Value v(Json::objectValue);
  v["body1"] = bcc.body1;
  v["body2"] = bcc.body2;
  v["collisionConf"] = scCollisionConfToJSON(bcc.collisionConf);
  return v;
}

inline void scCollisionsFromJSON(StanceConfig::Collisions & cs, const Json::Value & v)
{
  if(v.isMember("autoc"))
  {
    for(const auto & c : v["autoc"])
    {
      cs.autoc.push_back(scBodiesCollisionConfFromJSON(c));
    }
  }
  if(v.isMember("robotEnv"))
  {
    for(const auto & c : v["robotEnv"])
    {
      cs.robotEnv.push_back(scBodiesCollisionConfFromJSON(c));
    }
  }
  if(v.isMember("robotEnvContactFilter"))
  {
    for(const auto & f : v["robotEnvContactFilter"])
    {
      std::pair<std::string, std::string> key(f["surf1"].asString(),f["surf2"].asString());
      if(cs.robotEnvContactFilter.count(key) == 0)
      {
        cs.robotEnvContactFilter[key] = {};
      }
      for(const auto & filtered : f["filtered"])
      {
        cs.robotEnvContactFilter[key].push_back(std::pair<std::string, std::string>(filtered["body1"].asString(), filtered["body2"].asString()));
      }
    }
  }
}

inline Json::Value scCollisionsToJSON(const StanceConfig::Collisions & cs)
{
  Json::Value v(Json::objectValue);
  v["autoc"] = Json::Value(Json::arrayValue);
  for(const auto & bc : cs.autoc)
  {
    v["autoc"].append(scBodiesCollisionConfToJSON(bc));
  }
  v["robotEnv"] = Json::Value(Json::arrayValue);
  for(const auto & bc : cs.robotEnv)
  {
    v["robotEnv"].append(scBodiesCollisionConfToJSON(bc));
  }
  v["robotEnvContactFilter"] = Json::Value(Json::arrayValue);
  for(const auto & recf : cs.robotEnvContactFilter)
  {
    Json::Value vv(Json::objectValue);
    vv["surf1"] = recf.first.first;
    vv["surf2"] = recf.first.second;
    vv["filtered"] = Json::Value(Json::arrayValue);
    for(const auto & p : recf.second)
    {
      Json::Value vvv(Json::objectValue);
      vvv["body1"] = p.first;
      vvv["body2"] = p.second;
      vv["filtered"].append(vvv);
    }
    v["robotEnvContactFilter"].append(vv);
  }
  return v;
}

inline void StanceConfigFromJSON(StanceConfig & sc, const Json::Value & v)
{
  if(v.isMember("comTask"))
  {
    scCoMTaskFromJSON(sc.comTask, v["comTask"]);
  }
  if(v.isMember("comObj"))
  {
    scCoMObjFromJSON(sc.comObj, v["comObj"]);
  }
  if(v.isMember("postureTask"))
  {
    scPostureTaskFromJSON(sc.postureTask, v["postureTask"]);
  }
  if(v.isMember("contactTask"))
  {
    scContactTaskFromJSON(sc.contactTask, v["contactTask"]);
  }
  if(v.isMember("contactObj"))
  {
    scContactObjFromJSON(sc.contactObj, v["contactObj"]);
  }
  if(v.isMember("collisions"))
  {
    scCollisionsFromJSON(sc.collisions, v["collisions"]);
  }
}

inline Json::Value StanceConfigToJSON(const StanceConfig & sc)
{
  Json::Value v(Json::objectValue);
  v["comTask"] = scCoMTaskToJSON(sc.comTask);
  v["comObj"] = scCoMObjToJSON(sc.comObj);
  v["postureTask"] = scPostureTaskToJSON(sc.postureTask);
  v["contactTask"] = scContactTaskToJSON(sc.contactTask);
  v["contactObj"] = scContactObjToJSON(sc.contactObj);
  v["collisions"] = scCollisionsToJSON(sc.collisions);
  return v;
}

}
