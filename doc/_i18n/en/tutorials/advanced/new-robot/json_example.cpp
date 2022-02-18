#include <mc_rbdyn/RobotLoader.h>

std::string json_path = "/path/to/file.json";
auto rm_json = mc_rbdyn::RobotLoader::get_robot_module("json", json_path);

std::string yaml_path = "/path/to/file.yaml";
auto rm_yaml = mc_rbdyn::RobotLoader::get_robot_module("json", yaml_path);
