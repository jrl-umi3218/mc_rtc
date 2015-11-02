#include <mc_control/mc_global_controller.h>

#include <mc_rtc/config.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_robots/polaris_ranger.h>
#include <mc_robots/polaris_ranger_egress.h>

#include <json/json.h>
#include <fstream>

/* Note all service calls except for controller switches are implemented in mc_global_controller_services.cpp */

namespace mc_control
{

MCGlobalController::Configuration::Configuration(const std::string & path)
{
  Json::Value v;
  std::ifstream ifs(path);
  if(ifs.bad())
  {
    std::cerr << "Failed to open controller configuration file: " << path << std::endl;
  }
  try
  {
    ifs >> v;
  }
  catch(const std::runtime_error & exc)
  {
    std::cerr << "Failed to read configuration file" << std::endl;
    std::cerr << exc.what() << std::endl;
  }
  if(v.isMember("Enabled"))
  {
    for(const auto & cv : v["Enabled"])
    {
      enabled_controllers.push_back(cv.asString());
    }
  }
  if(v.isMember("Default"))
  {
    initial_controller = v["Default"].asString();
  }
  else
  {
    if(enabled_controllers.size())
    {
      initial_controller = enabled_controllers[0];
    }
  }
  /* Allow the user not to worry about Default if only one controller is enabled */
  if(enabled_controllers.size() == 1)
  {
    initial_controller = enabled_controllers[0];
  }
  if(v.isMember("Seq"))
  {
    if(v["Seq"].isMember("Env"))
    {
      if(v["Seq"]["Env"].isMember("Path"))
      {
        seq_env_path = v["Seq"]["Env"]["Path"].asString();
      }
      else
      {
        seq_env_path = "";
      }
      if(v["Seq"]["Env"].isMember("Name"))
      {
        seq_env_name = v["Seq"]["Env"]["Name"].asString();
      }
      else
      {
        seq_env_name = "";
      }
      if(v["Seq"]["Env"].isMember("Module"))
      {
        seq_env_module = v["Seq"]["Env"]["Module"].asString();
      }
      else
      {
        seq_env_module = "";
      }
    }
    if(v["Seq"].isMember("Plan"))
    {
      seq_plan = v["Seq"]["Plan"].asString();
    }
    if(v["Seq"].isMember("UseRealSensors"))
    {
      seq_use_real_sensors = v["Seq"]["UseRealSensors"].asBool();
    }
    if(v["Seq"].isMember("StartStance"))
    {
      seq_start_stance = v["Seq"]["StartStance"].asInt();
    }
    else
    {
      seq_start_stance = 0;
    }
  }
}

bool MCGlobalController::Configuration::enabled(const std::string & ctrl)
{
  return std::find(enabled_controllers.begin(), enabled_controllers.end(), ctrl) != enabled_controllers.end();
}

MCGlobalController::MCGlobalController()
: config(mc_rtc::CONF_PATH),
  posture_controller(0), body6d_controller(0), com_controller(0),
  seq_controller(0), driving_controller(0),
  egress_controller(0), egress_mrqp_controller(0),
  current_ctrl(NONE), next_ctrl(NONE),
  controller(0),
  next_controller(0)
{
  if(config.enabled("Posture"))
  {
    posture_controller.reset(new MCPostureController());
  }
  if(config.enabled("Body6d"))
  {
    body6d_controller.reset(new MCBody6dController());
  }
  if(config.enabled("CoM"))
  {
    com_controller.reset(new MCCoMController());
  }
  if(config.enabled("Seq"))
  {
    if(config.seq_env_name != "")
    {
      if(config.seq_env_path != "")
      {
        seq_controller.reset(new MCSeqController(config.seq_env_path, config.seq_env_name, std::string(mc_rtc::DATA_PATH) + config.seq_plan, config.seq_use_real_sensors, config.seq_start_stance));
      }
      else
      {
        seq_controller.reset(new MCSeqController(config.seq_env_name, std::string(mc_rtc::DATA_PATH) + config.seq_plan, config.seq_use_real_sensors, config.seq_start_stance));
      }
    }
    else if(config.seq_env_module != "")
    {
      if(config.seq_env_module == "Polaris")
      {
        seq_controller.reset(new MCSeqController(std::make_shared<mc_robots::PolarisRangerRobotModule>(false),
                                                 std::string(mc_rtc::DATA_PATH) + config.seq_plan,
                                                 config.seq_use_real_sensors, config.seq_start_stance));
      }
      else
      {
        std::cerr << "Unknown environment module provided (" << config.seq_env_module << ")" << std::endl;
      }
    }
    else
    {
      std::cerr << "Seq module enabled but no environment provided, Seq module not loaded" << std::endl;
    }
  }
  if(config.enabled("Driving"))
  {
    driving_controller.reset(new MCDrivingController({std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::PolarisRangerRobotModule()), std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::EnvRobotModule(mc_rtc::MC_ENV_DESCRIPTION_PATH, "ground"))}));
  }
  if(config.enabled("Egress"))
  {
    egress_controller.reset(new MCEgressController(mc_rtc::HRP2_DRC_DESCRIPTION_PATH, "polaris_ranger"));
  }
  if(config.enabled("EgressMRQP"))
  {
    egress_mrqp_controller.reset(new MCEgressMRQPController({std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::PolarisRangerEgressRobotModule()), std::shared_ptr<mc_rbdyn::RobotModule>(new mc_robots::EnvRobotModule(mc_rtc::MC_ENV_DESCRIPTION_PATH, "ground"))}));
  }
  if(config.enabled("BCISelfInteract"))
  {
    bci_self_interact_controller.reset(new MCBCISelfInteractController());
  }
  if(config.initial_controller == "Posture")
  {
    current_ctrl = POSTURE;
    controller = posture_controller.get();
  }
  if(config.initial_controller == "Body6d")
  {
    current_ctrl = BODY6D;
    controller = body6d_controller.get();
  }
  if(config.initial_controller == "CoM")
  {
    current_ctrl = COM;
    controller = com_controller.get();
  }
  if(config.initial_controller == "Seq")
  {
    current_ctrl = SEQ;
    controller = seq_controller.get();
  }
  if(config.initial_controller == "Driving")
  {
    current_ctrl = DRIVING;
    controller = driving_controller.get();
  }
  if(config.initial_controller == "Egress")
  {
    current_ctrl = EGRESS;
    controller = egress_controller.get();
  }
  if(config.initial_controller == "EgressMRQP")
  {
    current_ctrl = EGRESS_MRQP;
    controller = egress_mrqp_controller.get();
  }
  if(config.initial_controller == "BCISelfInteract")
  {
    current_ctrl = BCISELFINTERACT;
    controller = bci_self_interact_controller.get();
  }
  next_ctrl = current_ctrl;
  next_controller = 0;
  if(current_ctrl == NONE || controller == 0)
  {
    std::cerr << "No controller selected or selected controller is not enabled, please check your configuration file" << std::endl;
    throw("No controller enabled");
  }
}

void MCGlobalController::init(const std::vector<double> & initq)
{
  std::vector<std::vector<double>> q;
  /*FIXME Get the position/attitude of the robot? */
  //q.push_back({1,0,0,0,-0.275,-0.05,0.76});
  if(current_ctrl == SEQ)
  {
    q.push_back({0.999735735669,-0.000712226669955,0.0227020230143,-0.00354537868489,-0.364642021503,-0.00254821664029,0.762505884771});
  }
  else if(current_ctrl == EGRESS)
  {
    q.push_back({1, 0, 0, 0, 0, 0, 0.76});
  }
  else
  {
    q.push_back({1, 0, 0, 0, 0, 0, 0.76});
  }

  /* The OpenRTM components don't give q in the same order as the QP */
  for(size_t i = 0; i < 24; ++i) // until RARM_LINK7
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 32; i < 37; ++i) // RHAND
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 24; i < 32; ++i) // LARM_LINK*
  {
    q.push_back({initq[i]});
  }
  for(size_t i = 37; i < 42; ++i) // LHAND
  {
    q.push_back({initq[i]});
  }
  setGripperCurrentQ(initq[31], initq[23]);
  controller->reset({q});
}

void MCGlobalController::setSensorOrientation(const Eigen::Vector3d & rpy)
{
  controller->sensorOri = rpy;
}

void MCGlobalController::setEncoderValues(const std::vector<double> & eValues)
{
  controller->encoderValues = eValues;
}

void MCGlobalController::setWrenches(const std::vector< std::pair<Eigen::Vector3d, Eigen::Vector3d> > & wrenches)
{
  controller->setWrenches(wrenches);
}

void MCGlobalController::setActualGripperQ(double rQ, double lQ)
{
  controller->rgripper->setActualQ(rQ);
  controller->lgripper->setActualQ(lQ);
}

bool MCGlobalController::run()
{
  /* Check if we need to change the controller this time */
  if(next_controller)
  {
    std::cout << "Switching controllers" << std::endl;
    if(!running)
    {
      controller = next_controller;
    }
    else
    {
      /*XXX Need to be careful here */
      /*XXX Need to get the current contacts from the controller when needed */
      std::cout << "Reset with q[0]" << std::endl;
      std::cout << controller->robot().mbc().q[0][0] << " ";
      std::cout << controller->robot().mbc().q[0][1] << " ";
      std::cout << controller->robot().mbc().q[0][2] << " ";
      std::cout << controller->robot().mbc().q[0][3] << " ";
      std::cout << controller->robot().mbc().q[0][4] << " ";
      std::cout << controller->robot().mbc().q[0][5] << " ";
      std::cout << controller->robot().mbc().q[0][6] << std::endl;
      next_controller->lgripper->setCurrentQ(controller->lgripper->curPosition());
      next_controller->rgripper->setCurrentQ(controller->rgripper->curPosition());
      next_controller->reset({controller->robot().mbc().q});
      controller = next_controller;
    }
    next_controller = 0;
    current_ctrl = next_ctrl;
  }
  if(running)
  {
    bool r = controller->run();
    if(!r) { running = false; }
    return r;
  }
  else
  {
    return false;
  }
}

const QPResultMsg & MCGlobalController::send(const double & t)
{
  return controller->send(t);
}

const mc_rbdyn::Robot & MCGlobalController::robot()
{
  return controller->robot();
}

const std::vector<double> & MCGlobalController::gripperQ(bool lgripper)
{
  if(lgripper)
  {
    return controller->lgripper->q();
  }
  else
  {
    return controller->rgripper->q();
  }
}

void MCGlobalController::setGripperCurrentQ(double lQ, double rQ)
{
  controller->lgripper->setCurrentQ(lQ);
  controller->rgripper->setCurrentQ(rQ);
}

void MCGlobalController::setGripperTargetQ(double lQ, double rQ)
{
  controller->lgripper->setTargetQ(lQ);
  controller->rgripper->setTargetQ(rQ);
}

void MCGlobalController::setLGripperTargetQ(double lQ)
{
  controller->lgripper->setTargetQ(lQ);
}

void MCGlobalController::setRGripperTargetQ(double rQ)
{
  controller->rgripper->setTargetQ(rQ);
}

void MCGlobalController::setGripperOpenPercent(double lQ, double rQ)
{
  controller->lgripper->setTargetOpening(lQ);
  controller->rgripper->setTargetOpening(rQ);
}

bool MCGlobalController::EnableNextController(const std::string & name, const CurrentController & index, const std::shared_ptr<MCVirtualController> & ctrl)
{
  if(ctrl.get())
  {
    next_ctrl = index;
    if(current_ctrl != index)
    {
      next_controller = ctrl.get();
    }
    return true;
  }
  else
  {
    std::cout << name << " controller not enabled." << std::endl;
    return false;
  }
}

bool MCGlobalController::EnablePostureController()
{
  return EnableNextController("Posture", POSTURE, posture_controller);
}

bool MCGlobalController::EnableBody6dController()
{
  return EnableNextController("Body6d", BODY6D, body6d_controller);
}

bool MCGlobalController::EnableCoMController()
{
  return EnableNextController("CoM", COM, com_controller);
}

bool MCGlobalController::EnableSeqController()
{
  return EnableNextController("Seq", SEQ, seq_controller);
}

bool MCGlobalController::EnableDrivingController()
{
  return EnableNextController("Driving", DRIVING, driving_controller);
}

}
