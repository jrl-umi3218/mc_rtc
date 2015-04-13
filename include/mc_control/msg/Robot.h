#ifndef _H_MCCONTROLMSGROBOT_H_
#define _H_MCCONTROLMSGROBOT_H_

#include <vector>

namespace mc_control
{

struct RobotMsg
{
  std::vector<double> q;
  std::vector<double> alphaVec;
  Eigen::VectorXd alphaDVec;
};

}

#endif
