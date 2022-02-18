#include <mc_rbdyn/RobotLoader.h>

std::string path = "/path/to/description";
std::string name = "name";
auto env = mc_rbdyn::RobotLoader::get_robot_module("env", path, name);

// オブジェクトによってロボットと浮遊ベースが読み込まれる
auto object = mc_rbdyn::RobotLoader::get_robot_module("object", path, name);
