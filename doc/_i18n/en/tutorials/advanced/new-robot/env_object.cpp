#include <mc_rbdyn/RobotLoader.h>

std::string path = "/path/to/description";
std::string name = "name";
auto env = mc_rbdyn::RobotLoader::get_robot_module("env", path, name);

// object will load the robot with a floating base
auto object = mc_rbdyn::RobotLoader::get_robot_module("object", path, name);
