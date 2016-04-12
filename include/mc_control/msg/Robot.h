#ifndef _H_MCCONTROLMSGROBOT_H_
#define _H_MCCONTROLMSGROBOT_H_

#include <map>
#include <vector>
#include <Eigen/Core>

namespace mc_control
{

struct RobotMsg
{
  std::map<std::string, std::vector<double>> q;
  std::map<std::string, std::vector<double>> alphaVec;
  Eigen::VectorXd alphaDVec;
};

}

#endif
