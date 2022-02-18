#pragma once

#include <mc_rbdyn/RobotModule.h>
#include <mc_robots/api.h>

struct MC_ROBOTS_DLLAPI MyRobotModule : public mc_rbdyn::RobotModule
{
public:
  MyRobotModule(bool fixed);
};

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    // このモジュールによってエクスポートされるロボットの名前
    names = {"MyRobot", "MyRobotFixed"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name)
  {
    // 現時点ではサポートされているいずれかのロボットの名前を使用する必要がある
    if(name == "MyRobot")
    {
      return new MyRobot(false);
    }
    else
    {
      return new MyRobot(true);
    }
  }
}
