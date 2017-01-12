#include <mc_rbdyn/json/StanceConfig.h>
#include <fstream>

/* The role of this util is to write an initial configuration file */

int main(int argc, char * argv[])
{
  std::string out = "/tmp/config.json";
  if(argc > 1)
  {
    out = argv[1];
  }
  Json::Value v(Json::objectValue);
  Json::StyledWriter writer;
  Json::Value general(Json::objectValue);
  {
    mc_rbdyn::StanceConfig sc;
    sc.postureTask.stiffness = 0.1;
    sc.postureTask.weight = 10.0;
    sc.comTask.stiffness = 1.0;
    sc.comTask.extraStiffness = 1.0;
    sc.comTask.weight = 400.0;
    sc.comTask.targetSpeed = 0.001;
    sc.comObj.posThresh = 0.05,
    sc.comObj.velThresh = 0.0001;
    sc.comObj.comOffset = Eigen::Vector3d(0,0,0);
    general["CoMMove"] = mc_rbdyn::StanceConfigToJSON(sc);
  }
  {
    mc_rbdyn::StanceConfig sc;
    sc.postureTask.stiffness = 0.1;
    sc.postureTask.weight = 10.0;
    sc.comTask.stiffness = 3.0;
    sc.comTask.extraStiffness = 6.0;
    sc.comTask.weight = 500.0;
    sc.comTask.targetSpeed = 0.003;
    sc.comObj.comOffset = Eigen::Vector3d(0,0,0);
    sc.contactObj.posThresh = 0.03;
    sc.contactObj.velThresh = 0.005;
    sc.contactObj.preContactDist = 0.02;
    sc.contactTask.position.stiffness = 2.0;
    sc.contactTask.position.extraStiffness = 6.0;
    sc.contactTask.position.weight = 600.0;
    sc.contactTask.position.targetSpeed = 0.001;
    sc.contactTask.orientation.stiffness = 1.0;
    sc.contactTask.orientation.weight = 300.0;
    sc.contactTask.orientation.finalWeight = 1000.0;
    sc.contactTask.linVel.stiffness = 1.0;
    sc.contactTask.linVel.weight = 10000.0;
    sc.contactTask.linVel.speed = 0.02;
    sc.contactTask.waypointConf.thresh = 0.15;
    sc.contactTask.waypointConf.pos = mc_rbdyn::percentWaypoint(0.2, 1, 0.9, 0.2);
    sc.contactTask.collisionConf.iDist = 0.01;
    sc.contactTask.collisionConf.sDist = 0.005;
    sc.contactTask.collisionConf.damping = 0.05;
    general["ContactMove"] = mc_rbdyn::StanceConfigToJSON(sc);
  }
  {
    mc_rbdyn::StanceConfig sc;
    sc.postureTask.stiffness = 0.05;
    sc.postureTask.weight = 10.0;
    sc.comTask.stiffness = 0.5;
    sc.comTask.extraStiffness = 0.5;
    sc.comTask.weight = 500.0;
    sc.comTask.targetSpeed = 0.0005;
    sc.contactObj.posThresh = 0.03;
    sc.contactObj.velThresh = 0.05;
    sc.contactObj.adjustPosThresh = 0.05;
    sc.contactObj.adjustVelThresh = 0.02;
    sc.contactObj.adjustOriThresh = 0.1;
    sc.contactObj.adjustOriTBNWeight = Eigen::Vector3d(1,1,1);
    sc.contactObj.preContactDist = 0.02;
    sc.contactTask.position.stiffness = 0.25;
    sc.contactTask.position.extraStiffness = 1.0;
    sc.contactTask.position.weight = 600.0;
    sc.contactTask.position.targetSpeed = 0.0005;
    sc.contactTask.orientation.stiffness = 0.25;
    sc.contactTask.orientation.weight = 200.0;
    sc.contactTask.orientation.finalWeight = 1000.0;
    sc.contactTask.linVel.stiffness = 0.5;
    sc.contactTask.linVel.weight = 1000.0;
    sc.contactTask.linVel.speed = 0.02;
    sc.contactTask.waypointConf.thresh = 0.1;
    sc.contactTask.waypointConf.pos = mc_rbdyn::percentWaypoint(0.3, 0.8, 0.7, 0.2);
    sc.contactTask.collisionConf.iDist = 0.01;
    sc.contactTask.collisionConf.sDist = 0.005;
    sc.contactTask.collisionConf.damping = 0.05;
    general["GripperMove"] = mc_rbdyn::StanceConfigToJSON(sc);
  }
  {
    mc_rbdyn::StanceConfig sc;
    sc.collisions.autoc.push_back({"RLEG_LINK2", "LLEG_LINK2", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"RLEG_LINK3", "LLEG_LINK3", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"RLEG_LINK5", "LLEG_LINK5", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"RLEG_LINK5", "LLEG_LINK3", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"LLEG_LINK5", "RLEG_LINK3", {0.05, 0.01, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "RLEG_LINK3", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "RLEG_LINK2", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "RLEG_LINK3", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "LLEG_LINK2", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "LLEG_LINK3", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "CHEST_LINK1", {0.1, 0.05, 0.0}});
    sc.collisions.autoc.push_back({"RARM_LINK6", "BODY", {0.1, 0.05, 0.0}});
    sc.collisions.robotEnv.push_back({"RARM_LINK6", "stair_step2", {0.1, 0.05, 0.0}});
    sc.collisions.robotEnv.push_back({"RARM_LINK6", "stair_step3", {0.1, 0.05, 0.0}});
    sc.collisions.robotEnv.push_back({"RARM_LINK6", "platform", {0.1, 0.05, 0.0}});
    general["Collisions"] = mc_rbdyn::scCollisionsToJSON(sc.collisions);
  }
  v["General"] = general;

  Json::Value stepByStep(Json::arrayValue);
  {
    mc_rbdyn::StanceConfig sc;
    sc.comObj.comOffset = Eigen::Vector3d(0.05, 0.0,0.0);

    Json::Value step = mc_rbdyn::StanceConfigToJSON(sc);
    step["type"] = "add"; /* Legit values identity/add/remove */
    step["r1Surface"] = "LeftGripper";
    step["r2Surface"] = "StairLeftRung1";
    stepByStep.append(step);
  }
  {
    mc_rbdyn::StanceConfig sc;
    sc.comObj.comOffset = Eigen::Vector3d(1e-6, 0.0,0.0);

    Json::Value step = mc_rbdyn::StanceConfigToJSON(sc);
    step["type"] = "add"; /* Legit values identity/add/remove */
    step["r1Surface"] = "LeftGripper";
    step["r2Surface"] = "StairLeftRung2";
    stepByStep.append(step);
  }
  v["StepByStep"] = stepByStep;

  std::string output = writer.write(v);
  std::ofstream ofs(out);
  ofs << output;
  std::cout << "Initial config written to " << out << std::endl;
  return 0;
}
