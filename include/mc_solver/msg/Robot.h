#pragma once

#include <map>
#include <vector>
#include <Eigen/Core>

namespace mc_solver
{

struct RobotMsg
{
  std::map<std::string, std::vector<double>> q;
  std::map<std::string, std::vector<double>> alphaVec;
  Eigen::VectorXd alphaDVec;
};

}
