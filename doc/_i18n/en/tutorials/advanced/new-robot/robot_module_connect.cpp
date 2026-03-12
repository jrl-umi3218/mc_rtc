#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>

// Define a "robot_support" robot from a "box" visual description
auto boxYaml =
    R"(
  name: robot_support
  origin:
    translation: [0, 0, 0]
    rotation: [0, 0, 0]
  material:
    color:
      r: 1.0
      g: 0.0
      b: 0.0
      a: 1.0
  geometry:
    box:
      size: [0.7, 1.2, 1.2]
  inertia:
    mass: 50
  fixed: true
  )";
auto boxConfig = mc_rtc::Configuration::fromYAMLData(boxYaml);

// Create a "robot_support" robot module from the visual description above
auto supportRm = mc_rbdyn::robotModuleFromVisual("robot_support", boxConfig);
// Load the panda robot module from an existing robot module
auto pandaRm = mc_rbdyn::RobotLoader::get_robot_module("PandaDefault");

// Transformation between the base of the "world" link of PandaDefault module and the robot_support link
auto box_to_robot = sva::PTransformd(Eigen::Vector3d{0., 0., -1.2});

// Load a new robot module by connecting the existing "PandaDefault" module to our "box" module
auto pandaOnBoxRm = supportRm->connect(*pandaRm, // connect to this other robot module (panda)
                                       "robot_support", // reference link on our module
                                       "world", // to connect to this link on other robot module (panda)
                                       "", // optional prefix to add to link names
                                       mc_rbdyn::RobotModule::ConnectionParameters{}
                                           .name(robot_name)
                                           .X_other_connection(box_to_robot) // connection parameters
);

// Add robot to the controller, where ctl is an mc_control::MCController
auto robot = ctl.loadRobot(*pandaOnBoxRm);
