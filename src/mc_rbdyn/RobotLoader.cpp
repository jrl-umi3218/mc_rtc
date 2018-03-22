#include <mc_rbdyn/RobotLoader.h>

std::unique_ptr<mc_rtc::ObjectLoader<mc_rbdyn::RobotModule>> mc_rbdyn::RobotLoader::robot_loader;
bool mc_rbdyn::RobotLoader::enable_sandbox_ = false;
bool mc_rbdyn::RobotLoader::verbose_ = false;
std::mutex mc_rbdyn::RobotLoader::mtx{};
